"""Validated per-hand grasp TCP calibration shared by IK and MuJoCo."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

from .robot_profiles import RobotProfile


DEFAULT_TOOL_POSE_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "tool_pose_offset.json"
)


@dataclass(frozen=True)
class ToolPose:
    parent_frame: str
    translation_m: tuple[float, float, float]
    rpy_rad: tuple[float, float, float]


@dataclass(frozen=True)
class ToolPoseConfig:
    source_path: Path
    left: ToolPose
    right: ToolPose

    def for_side(self, side: str) -> ToolPose:
        if side == "left":
            return self.left
        if side == "right":
            return self.right
        raise ValueError(f"unsupported arm side: {side!r}")


def _finite_triplet(raw: object, label: str) -> tuple[float, float, float]:
    if not isinstance(raw, list) or len(raw) != 3:
        raise ValueError(f"{label} must contain three values")
    values = tuple(float(value) for value in raw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain finite values")
    return values


def _read_side(document: dict[str, object], side: str) -> ToolPose:
    raw = document.get(side)
    if not isinstance(raw, dict):
        raise ValueError(f"tool pose calibration is missing {side!r}")
    parent = raw.get("parent_frame")
    if not isinstance(parent, str) or not parent:
        raise ValueError(f"{side}.parent_frame must be a non-empty string")
    translation = _finite_triplet(raw.get("translation_m"), f"{side}.translation_m")
    rotation = _finite_triplet(raw.get("rpy_rad"), f"{side}.rpy_rad")
    if max(abs(value) for value in translation) > 0.50:
        raise ValueError(f"{side}.translation_m exceeds the 0.50 m safety limit")
    if max(abs(value) for value in rotation) > 2.0 * math.pi:
        raise ValueError(f"{side}.rpy_rad exceeds the 2*pi safety limit")
    return ToolPose(parent, translation, rotation)


def load_tool_pose_config(
    profile: RobotProfile,
    path: Path | None = None,
) -> ToolPoseConfig:
    source = (path or DEFAULT_TOOL_POSE_PATH).expanduser().resolve()
    document = json.loads(source.read_text(encoding="utf-8"))
    if document.get("schema_version") != 1:
        raise ValueError("tool pose calibration schema_version must be 1")
    left = _read_side(document, "left")
    right = _read_side(document, "right")
    if left.parent_frame != profile.left_ee_frame:
        raise ValueError(
            f"left parent frame {left.parent_frame!r} does not match {profile.left_ee_frame!r}"
        )
    if right.parent_frame != profile.right_ee_frame:
        raise ValueError(
            f"right parent frame {right.parent_frame!r} does not match {profile.right_ee_frame!r}"
        )
    return ToolPoseConfig(source, left, right)
