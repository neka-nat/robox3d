"""Capture box3d log output.

Registers a callback with b3SetLogFcn so the solver's "unstable" warnings
(velocity NaN) can be detected and converted into Python warnings. The callback
is kept alive by a module-level reference (a GC'd callback would crash).
"""

from __future__ import annotations

from .._ffi import ffi, lib

_messages: list[str] = []


@ffi.callback("void(const char*)")
def _log_callback(message):
    _messages.append(ffi.string(message).decode(errors="replace"))


lib.b3SetLogFcn(_log_callback)


def drain() -> list[str]:
    """Pop the accumulated log messages and clear the buffer."""
    msgs = _messages[:]
    _messages.clear()
    return msgs


def pending() -> int:
    return len(_messages)
