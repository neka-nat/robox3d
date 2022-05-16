"""Layer 4: visualization. WebSocket pose streaming plus recording/replay.

The web viewer (React Three Fiber) is served automatically by VizServer when
the bundled build exists (``tools/build_viewer.py``). For viewer development,
run it from source: ``cd viewer && pnpm install && pnpm dev``.
"""

from .protocol import decode_pose_frame, encode_pose_frame, scene_message
from .record import PoseRecorder, load_recording, replay
from .server import VizServer

__all__ = [
    "VizServer",
    "PoseRecorder",
    "load_recording",
    "replay",
    "scene_message",
    "encode_pose_frame",
    "decode_pose_frame",
]
