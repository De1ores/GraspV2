"""Runtime logging controls for ROS 2 processes used by GraspV2."""

from __future__ import annotations

import ctypes
import os


_FASTDDS_SET_VERBOSITY_SYMBOL = (
    "_ZN8eprosima7fastdds3dds3Log12SetVerbosityENS2_4KindE"
)
_FASTDDS_LEVELS = {
    "error": 0,
    "warning": 1,
    "info": 2,
}


class RosLoggingError(RuntimeError):
    """Raised when a requested ROS middleware logging mode is invalid."""


def configure_fastdds_logging(level: str | None = None) -> bool:
    """Limit Fast DDS diagnostics before the first participant is created.

    The robot's Fast DDS build labels routine discovery and endpoint matching
    as warnings, producing thousands of terminal lines for a short command.
    GraspV2 therefore keeps only Fast DDS errors by default. Set
    ``GRASPV2_FASTDDS_LOG_LEVEL=warning`` to restore vendor warnings,
    ``info`` for full diagnostics, or ``default`` to leave the library alone.

    Returns ``True`` when the Fast DDS runtime was configured. A missing
    Fast DDS library is not an error because offline planning and alternate
    RMW implementations must remain usable.
    """

    selected = (
        level
        if level is not None
        else os.environ.get("GRASPV2_FASTDDS_LOG_LEVEL", "error")
    )
    normalized = selected.strip().lower()
    if normalized == "default":
        return False
    if normalized not in _FASTDDS_LEVELS:
        allowed = ", ".join((*_FASTDDS_LEVELS, "default"))
        raise RosLoggingError(
            "GRASPV2_FASTDDS_LOG_LEVEL must be one of " + allowed
        )

    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    if rmw and "fastrtps" not in rmw:
        return False

    for library_name in ("libfastrtps.so.2.6", "libfastrtps.so"):
        try:
            library = ctypes.CDLL(library_name)
            set_verbosity = getattr(
                library,
                _FASTDDS_SET_VERBOSITY_SYMBOL,
            )
        except (AttributeError, OSError):
            continue
        set_verbosity.argtypes = [ctypes.c_int]
        set_verbosity.restype = None
        set_verbosity(_FASTDDS_LEVELS[normalized])
        return True
    return False
