"""Pose recording and replay.

File format (.rbx):
    [8-byte magic "RBX3DR01"]
    [u32 scene JSON byte length][scene JSON (utf-8)]
    repeated frames: [u32 payload length][pose frame (same format as protocol.py)]

Recordings use the same frames as live streaming, so a replay can be sent to
the same viewer as VizServer.
"""

from __future__ import annotations

import struct
import time
from pathlib import Path

from ..core.batch import BodyGroup
from ..core.world import World
from .protocol import decode_pose_frame, encode_pose_frame, scene_message

MAGIC = b"RBX3DR01"
_U32 = struct.Struct("<I")


class PoseRecorder:
    """Records poses; just call update() from the sim loop."""

    def __init__(self, world: World, path):
        self.world = world
        self.path = Path(path)
        self._group = BodyGroup(world.bodies)
        self._f = self.path.open("wb")
        self._f.write(MAGIC)
        scene = scene_message(world).encode()
        self._f.write(_U32.pack(len(scene)))
        self._f.write(scene)

    def update(self) -> None:
        frame = encode_pose_frame(self.world.time, self._group.poses())
        self._f.write(_U32.pack(len(frame)))
        self._f.write(frame)

    def close(self) -> None:
        self._f.close()

    def __enter__(self) -> "PoseRecorder":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def load_recording(path) -> tuple[str, list[bytes]]:
    """Load a recording file into (scene JSON, list of frames)."""
    data = Path(path).read_bytes()
    if data[:8] != MAGIC:
        raise ValueError(f"{path} is not a robox3d recording file")
    offset = 8
    (scene_len,) = _U32.unpack_from(data, offset)
    offset += 4
    scene_json = data[offset : offset + scene_len].decode()
    offset += scene_len
    frames = []
    while offset < len(data):
        (n,) = _U32.unpack_from(data, offset)
        offset += 4
        frames.append(data[offset : offset + n])
        offset += n
    return scene_json, frames


def replay(path, host: str = "127.0.0.1", port: int = 8765, loop: bool = True) -> None:
    """Stream a recording file to the viewer in real time (Ctrl+C to quit).

    The same port also serves the bundled web viewer over HTTP, so just open
    http://<host>:<port> in a browser.
    """
    import asyncio

    from websockets.asyncio.server import serve

    from .server import http_response

    scene_json, frames = load_recording(path)
    times = [decode_pose_frame(f)[0] for f in frames]
    print(
        f"Replaying {len(frames)} frames ({times[-1] - times[0]:.1f}s) — "
        f"open http://{host}:{port} in a browser"
    )

    async def handler(ws):
        await ws.send(scene_json)
        while True:
            start = time.monotonic()
            for frame, t in zip(frames, times):
                await asyncio.sleep(max(0.0, (t - times[0]) - (time.monotonic() - start)))
                await ws.send(frame)
            if not loop:
                break

    async def main():
        async with serve(handler, host, port, process_request=http_response):
            await asyncio.Future()

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    import sys

    replay(sys.argv[1])
