"""WebSocket visualization server (layer 4).

Physics runs on the sim thread (the caller); streaming runs on a background
asyncio loop. The sim loop just calls server.update() after step(). The
broadcaster sends only the latest frame at the configured fps (latest-wins,
dropped frames are acceptable).

The same port also serves the bundled web viewer over plain HTTP: non-WebSocket
GET requests receive static files from ``robox3d/viz/static`` (populated by
``tools/build_viewer.py`` or shipped inside the wheel). Open ``server.url`` in
a browser and the viewer connects back automatically.

Commands from the viewer (JSON) are queued and applied on the sim thread during
update() (the engine is not thread-safe). Passing robot= enables automatic
handling of "set_target" commands (joint sliders).

Usage:
    server = VizServer(world, robot=robot)
    server.start()
    while running:
        world.step(dt)
        server.update()
    server.stop()
"""

from __future__ import annotations

import asyncio
import json
import queue
import threading
import warnings
from http import HTTPStatus
from pathlib import Path
from typing import Callable

from ..core.batch import BodyGroup
from ..core.world import World
from .protocol import encode_pose_frame, robot_target_values, scene_dict

# Static viewer assets bundled with the package (created by tools/build_viewer.py).
STATIC_DIR = Path(__file__).parent / "static"

_MIME_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript",
    ".mjs": "text/javascript",
    ".css": "text/css",
    ".json": "application/json",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
    ".png": "image/png",
    ".map": "application/json",
    ".wasm": "application/wasm",
    ".txt": "text/plain; charset=utf-8",
}

_FALLBACK_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>robox3d</title></head>
<body style="font-family: sans-serif; background: #15171c; color: #cdd3dc; padding: 2em;">
<h2>robox3d viz server is running</h2>
<p>The WebSocket stream is live on this port, but the bundled web viewer was
not found in this installation.</p>
<p>To use the viewer, either:</p>
<ul>
<li>build it once: <code>python tools/build_viewer.py</code> (requires pnpm), or</li>
<li>run the dev viewer: <code>cd viewer &amp;&amp; pnpm install &amp;&amp; pnpm dev</code>
and open <a href="http://localhost:5173" style="color:#4c9be8">http://localhost:5173</a>.</li>
</ul>
</body></html>"""


def http_response(connection, request, static_dir: Path | None = None):
    """Serve the bundled viewer for non-WebSocket requests.

    Returns None for WebSocket upgrade requests so the handshake proceeds.
    Intended as the ``process_request`` callback of ``websockets.serve``.
    """
    if request.headers.get("Upgrade", "").lower() == "websocket":
        return None

    from websockets.datastructures import Headers
    from websockets.http11 import Response

    root = (static_dir if static_dir is not None else STATIC_DIR).resolve()
    raw_path = request.path.split("?", 1)[0]
    rel = raw_path.lstrip("/") or "index.html"
    file = (root / rel).resolve()
    # Reject path traversal and anything outside the static root.
    if not file.is_relative_to(root):
        return connection.respond(HTTPStatus.FORBIDDEN, "Forbidden\n")
    if file.is_dir():
        file = file / "index.html"

    if not file.is_file():
        if raw_path in ("/", "/index.html"):
            body = _FALLBACK_PAGE.encode()
            headers = Headers(
                [
                    ("Content-Type", "text/html; charset=utf-8"),
                    ("Content-Length", str(len(body))),
                ]
            )
            return Response(HTTPStatus.OK, "OK", headers, body)
        return connection.respond(HTTPStatus.NOT_FOUND, "Not found\n")

    body = file.read_bytes()
    ctype = _MIME_TYPES.get(file.suffix.lower(), "application/octet-stream")
    headers = Headers(
        [
            ("Content-Type", ctype),
            ("Content-Length", str(len(body))),
            ("Cache-Control", "no-cache"),
        ]
    )
    return Response(HTTPStatus.OK, "OK", headers, body)


class VizServer:
    def __init__(
        self,
        world: World,
        robot=None,
        host: str = "127.0.0.1",
        port: int = 8765,
        fps: float = 60.0,
        on_command: Callable[[dict], None] | None = None,
    ):
        try:
            import websockets  # noqa: F401
        except ImportError as e:
            raise ImportError(
                "VizServer requires websockets: uv add 'robox3d[viz]' "
                "or pip install websockets"
            ) from e
        self.world = world
        self.robot = robot
        self.host = host
        self.port = port
        self.fps = fps
        self.on_command = on_command
        self._commands: queue.Queue[dict] = queue.Queue()
        self._scene: dict | None = None
        self._group: BodyGroup | None = None
        self._latest: bytes | None = None  # written by sim thread, read by broadcaster
        # Joint-target snapshot so late-joining clients get current slider values.
        self._joint_values: list[float] | None = None
        self._values_time = float("-inf")
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._stop_event: asyncio.Event | None = None
        self._started = threading.Event()
        self._start_error: BaseException | None = None

    # ------------------------------------------------------------ sim thread side

    def start(self) -> None:
        """Freeze the scene and start the streaming thread.

        Call after all bodies have been created.
        """
        robots = [self.robot] if self.robot is not None else []
        self._scene = scene_dict(self.world, robots=robots)
        self._group = BodyGroup(self.world.bodies)
        self.update()
        self._thread = threading.Thread(target=self._run, daemon=True, name="robox3d-viz")
        self._thread.start()
        if not self._started.wait(timeout=5.0):
            raise RuntimeError("VizServer startup timed out")
        if self._start_error is not None:
            raise RuntimeError(
                f"VizServer failed to start on {self.host}:{self.port} "
                f"({self._start_error}) — is another server using the port?"
            ) from self._start_error

    def update(self) -> None:
        """Capture the latest poses and apply pending viewer commands.

        Call from the sim thread after world.step().
        """
        while True:
            try:
                cmd = self._commands.get_nowait()
            except queue.Empty:
                break
            self._apply_command(cmd)
        self._latest = encode_pose_frame(self.world.time, self._group.poses())
        # Snapshot joint targets (throttled) on the sim thread — the server
        # thread must not call into the engine while the sim is stepping.
        if self.robot is not None and self.world.time - self._values_time >= 0.1:
            self._joint_values = robot_target_values(self.robot)
            self._values_time = self.world.time

    def _apply_command(self, cmd: dict) -> None:
        if cmd.get("type") == "set_target" and self.robot is not None:
            from ..core.joints import PrismaticJoint

            name = cmd.get("joint")
            joint = self.robot.joints.get(name)
            if joint is None:
                return
            value = float(cmd.get("value", 0.0))
            limits = self.robot.position_limits.get(name)
            if limits is not None:
                value = min(max(value, limits[0]), limits[1])
            if isinstance(joint, PrismaticJoint):
                joint.target_translation = value
            else:
                joint.target_angle = value
        elif self.on_command is not None:
            self.on_command(cmd)

    def stop(self) -> None:
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._stop_event.set)
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    @property
    def url(self) -> str:
        """Browser URL. The same port serves the viewer (HTTP) and poses (WS)."""
        return f"http://{self.host}:{self.port}"

    @property
    def ws_url(self) -> str:
        return f"ws://{self.host}:{self.port}"

    # ------------------------------------------------------------ server thread side

    def _run(self) -> None:
        try:
            asyncio.run(self._serve())
        except BaseException as e:  # surfaced by start() on the sim thread
            self._start_error = e
            self._started.set()

    async def _serve(self) -> None:
        from websockets.asyncio.server import broadcast, serve

        self._loop = asyncio.get_running_loop()
        self._stop_event = asyncio.Event()
        clients: set = set()

        async def handler(ws):
            # Refresh slider values so late-joining clients see current targets.
            scene = self._scene
            values = self._joint_values
            if scene is not None and values is not None and scene["robots"]:
                for jd, value in zip(scene["robots"][0]["joints"], values):
                    jd["value"] = value
            await ws.send(json.dumps(scene))
            clients.add(ws)
            try:
                async for message in ws:
                    if isinstance(message, str):
                        try:
                            self._commands.put(json.loads(message))
                        except json.JSONDecodeError:
                            warnings.warn(f"Invalid command: {message[:100]}", stacklevel=2)
            finally:
                clients.discard(ws)

        async def broadcaster():
            interval = 1.0 / self.fps
            last_sent: bytes | None = None
            while not self._stop_event.is_set():
                frame = self._latest
                if frame is not None and frame is not last_sent and clients:
                    broadcast(clients, frame)
                    last_sent = frame
                try:
                    await asyncio.wait_for(self._stop_event.wait(), timeout=interval)
                except (TimeoutError, asyncio.TimeoutError):  # asyncio.TimeoutError on 3.10
                    pass

        async with serve(handler, self.host, self.port, process_request=http_response):
            self._started.set()
            await broadcaster()
