"""Pure-Python contract for a locally planned X2 edge-execution bundle."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path


SCHEMA_VERSION = 1
BUNDLE_KIND = "graspv2-x2-edge-execution"
ANIMATION_FILENAME = "grasp_animation.csv"


class EdgeManifestError(ValueError):
    """Raised before an untrusted or incomplete edge bundle can be used."""


@dataclass(frozen=True)
class EdgeGripperEvent:
    time_s: float
    position: float
    label: str


@dataclass(frozen=True)
class EdgeExecutionManifest:
    source: Path
    animation_path: Path
    animation_sha256: str
    duration_s: float
    maximum_arm_velocity_rad_s: float
    initial_gripper_position: float
    initial_gripper_duration_s: float
    gripper_events: tuple[EdgeGripperEvent, ...]
    robot_profile: str
    arm_side: str
    target_class: str


def sha256_file(path: Path) -> str:
    """Hash a regular file without loading a potentially large CSV at once."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _object(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise EdgeManifestError(f"{label} must be an object")
    return value


def _text(value: object, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise EdgeManifestError(f"{label} must be a non-empty string")
    return value


def _number(value: object, label: str, *, minimum: float = 0.0) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EdgeManifestError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result) or result < minimum:
        raise EdgeManifestError(
            f"{label} must be finite and at least {minimum}"
        )
    return result


def load_edge_manifest(
    path: Path, *, verify_files: bool = True
) -> EdgeExecutionManifest:
    """Load and strictly validate the robot-visible execution contract."""

    source = path.expanduser().resolve()
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise EdgeManifestError(
            f"execution manifest does not exist: {source}"
        ) from error
    except (OSError, json.JSONDecodeError) as error:
        raise EdgeManifestError(f"cannot read execution manifest: {error}") from error
    root = _object(document, "execution manifest")
    if root.get("schema_version") != SCHEMA_VERSION:
        raise EdgeManifestError(
            f"unsupported execution manifest schema: {root.get('schema_version')!r}"
        )
    if root.get("kind") != BUNDLE_KIND:
        raise EdgeManifestError(
            f"unexpected execution bundle kind: {root.get('kind')!r}"
        )

    robot = _object(root.get("robot"), "robot")
    profile = _text(robot.get("profile"), "robot.profile")
    side = _text(robot.get("arm_side"), "robot.arm_side")
    if profile != "ultra" or side != "right":
        raise EdgeManifestError(
            "edge execution only supports the X2 Ultra right OmniPicker"
        )
    target_class = _text(root.get("target_class"), "target_class")

    animation = _object(root.get("animation"), "animation")
    if animation.get("return_path_enabled") is not True:
        raise EdgeManifestError("animation must include its MC return path")
    filename = _text(animation.get("file"), "animation.file")
    if filename != ANIMATION_FILENAME or Path(filename).name != filename:
        raise EdgeManifestError(
            f"animation.file must be exactly {ANIMATION_FILENAME!r}"
        )
    checksum = _text(animation.get("sha256"), "animation.sha256").lower()
    if len(checksum) != 64 or any(
        character not in "0123456789abcdef" for character in checksum
    ):
        raise EdgeManifestError("animation.sha256 must be 64 lowercase hex digits")
    duration = _number(animation.get("duration_s"), "animation.duration_s")
    if duration <= 0.0:
        raise EdgeManifestError("animation.duration_s must be positive")
    maximum_velocity = _number(
        animation.get("maximum_arm_velocity_rad_s"),
        "animation.maximum_arm_velocity_rad_s",
    )
    if maximum_velocity <= 0.0 or maximum_velocity > 0.5:
        raise EdgeManifestError(
            "animation maximum arm velocity must be within (0, 0.5] rad/s"
        )

    initial = _object(animation.get("initial_gripper"), "animation.initial_gripper")
    initial_position = _number(
        initial.get("position"), "animation.initial_gripper.position"
    )
    if initial_position > 1.0:
        raise EdgeManifestError("initial gripper position must be within [0, 1]")
    initial_duration = _number(
        initial.get("duration_s"), "animation.initial_gripper.duration_s"
    )
    if initial_duration <= 0.0:
        raise EdgeManifestError("initial gripper duration must be positive")

    raw_events = animation.get("gripper_events")
    if not isinstance(raw_events, list) or not raw_events:
        raise EdgeManifestError("animation.gripper_events must be a non-empty list")
    events: list[EdgeGripperEvent] = []
    for index, raw_event in enumerate(raw_events):
        event = _object(raw_event, f"animation.gripper_events[{index}]")
        time_s = _number(event.get("time_s"), f"gripper event {index} time_s")
        position = _number(
            event.get("position"), f"gripper event {index} position"
        )
        if position > 1.0:
            raise EdgeManifestError(
                f"gripper event {index} position must be within [0, 1]"
            )
        if time_s > duration:
            raise EdgeManifestError(
                f"gripper event {index} occurs after the animation ends"
            )
        label = _text(event.get("label"), f"gripper event {index} label")
        if events and time_s <= events[-1].time_s:
            raise EdgeManifestError("gripper event times must be strictly increasing")
        events.append(EdgeGripperEvent(time_s, position, label))

    limitations = _object(
        root.get("execution_limitations"), "execution_limitations"
    )
    if limitations.get("atomic_mc_animation") is not True:
        raise EdgeManifestError("edge execution must declare atomic MC playback")
    if limitations.get("mid_motion_visual_gates") is not False:
        raise EdgeManifestError(
            "edge execution must explicitly declare missing mid-motion visual gates"
        )

    animation_path = source.parent / filename
    if verify_files:
        if not animation_path.is_file():
            raise EdgeManifestError(f"animation file does not exist: {animation_path}")
        actual_checksum = sha256_file(animation_path)
        if actual_checksum != checksum:
            raise EdgeManifestError(
                "animation SHA-256 mismatch: "
                f"manifest={checksum}, actual={actual_checksum}"
            )

    return EdgeExecutionManifest(
        source=source,
        animation_path=animation_path,
        animation_sha256=checksum,
        duration_s=duration,
        maximum_arm_velocity_rad_s=maximum_velocity,
        initial_gripper_position=initial_position,
        initial_gripper_duration_s=initial_duration,
        gripper_events=tuple(events),
        robot_profile=profile,
        arm_side=side,
        target_class=target_class,
    )
