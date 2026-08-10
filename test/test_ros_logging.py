"""Tests for GraspV2's middleware log filtering."""

from types import SimpleNamespace

import pytest

from graspv2 import ros_logging


class _FakeVerbosity:
    def __init__(self) -> None:
        self.argtypes = None
        self.restype = object()
        self.calls: list[int] = []

    def __call__(self, level: int) -> None:
        self.calls.append(level)


def test_fastdds_defaults_to_error_only(monkeypatch: pytest.MonkeyPatch) -> None:
    verbosity = _FakeVerbosity()
    library = SimpleNamespace()
    setattr(library, ros_logging._FASTDDS_SET_VERBOSITY_SYMBOL, verbosity)
    monkeypatch.setattr(ros_logging.ctypes, "CDLL", lambda _name: library)
    monkeypatch.delenv("GRASPV2_FASTDDS_LOG_LEVEL", raising=False)
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

    assert ros_logging.configure_fastdds_logging() is True
    assert verbosity.calls == [0]
    assert verbosity.argtypes == [ros_logging.ctypes.c_int]
    assert verbosity.restype is None


def test_fastdds_warning_mode_can_be_restored(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    verbosity = _FakeVerbosity()
    library = SimpleNamespace()
    setattr(library, ros_logging._FASTDDS_SET_VERBOSITY_SYMBOL, verbosity)
    monkeypatch.setattr(ros_logging.ctypes, "CDLL", lambda _name: library)

    assert ros_logging.configure_fastdds_logging("warning") is True
    assert verbosity.calls == [1]


def test_fastdds_default_and_other_rmw_are_untouched(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def unexpected_load(_name: str):
        raise AssertionError("Fast DDS library should not be loaded")

    monkeypatch.setattr(ros_logging.ctypes, "CDLL", unexpected_load)
    assert ros_logging.configure_fastdds_logging("default") is False
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    assert ros_logging.configure_fastdds_logging("error") is False


def test_invalid_fastdds_level_is_rejected() -> None:
    with pytest.raises(ros_logging.RosLoggingError, match="must be one of"):
        ros_logging.configure_fastdds_logging("debug")
