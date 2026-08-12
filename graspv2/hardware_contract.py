"""ROS-independent contracts for the X2 AimDK hardware boundary.

The planner and simulator must remain usable without ROS.  This module keeps
all topic names, tuning, and command validation independent from ``rclpy`` so
the live adapter is a thin, replaceable boundary rather than a dependency of
the planning stack.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Mapping, Protocol, Sequence

from .robot_profiles import (
    INSTALLED_GRIPPER_SIDE,
    LEFT_ARM_7,
    RIGHT_ARM_7,
    RobotProfile,
)


ROOT = Path(__file__).resolve().parents[1]
_SOURCE_HARDWARE_CONFIG_PATH = ROOT / "config" / "x2_aimdk_hardware.json"
_INSTALLED_HARDWARE_CONFIG_PATH = (
    Path(sys.prefix) / "share" / "graspv2" / "config" / "x2_aimdk_hardware.json"
)
DEFAULT_HARDWARE_CONFIG_PATH = (
    _SOURCE_HARDWARE_CONFIG_PATH
    if _SOURCE_HARDWARE_CONFIG_PATH.is_file()
    else _INSTALLED_HARDWARE_CONFIG_PATH
)


class HardwareContractError(ValueError):
    """Raised before a malformed command can reach the ROS adapter."""


@dataclass(frozen=True)
class AimDKTopics:
    """AimDK topic names used by graspV2.

    Defaults follow the X2 AimDK v0.9/v1.0 manuals.  They are configurable so
    a firmware namespace change does not leak into planning or perception.
    """

    arm_command: str = "/aima/hal/joint/arm/command"
    arm_state: str = "/aima/hal/joint/arm/state"
    waist_command: str = "/aima/hal/joint/waist/command"
    waist_state: str = "/aima/hal/joint/waist/state"
    head_command: str = "/aima/hal/joint/head/command"
    head_state: str = "/aima/hal/joint/head/state"
    hand_command: str = "/aima/hal/joint/hand/command"
    hand_state: str = "/aima/hal/joint/hand/state"
    upper_body_command: str = "/mc/upper_body_command"
    rgb_image: str = "/aima/hal/sensor/rgbd_head_front/rgb_image"
    depth_image: str = "/aima/hal/sensor/rgbd_head_front/depth_image"
    rgb_camera_info: str = (
        "/aima/hal/sensor/rgbd_head_front/rgb_camera_info"
    )
    depth_camera_info: str = (
        "/aima/hal/sensor/rgbd_head_front/depth_camera_info"
    )


@dataclass(frozen=True)
class AimDKServices:
    """Motion-controller services used by upper-body split mode."""

    get_mc_action: str = "/aimdk_5Fmsgs/srv/GetMcAction"
    set_mc_action: str = "/aimdk_5Fmsgs/srv/SetMcAction"
    get_current_input_source: str = (
        "/aimdk_5Fmsgs/srv/GetCurrentInputSource"
    )
    set_mc_input_source: str = "/aimdk_5Fmsgs/srv/SetMcInputSource"


@dataclass(frozen=True)
class UpperBodyTuning:
    """Conservative MC split-mode and direct HAL control settings."""

    command_rate_hz: float = 50.0
    arm_stiffness: float = 20.0
    arm_damping: float = 2.0
    waist_stiffness: float = 20.0
    waist_damping: float = 4.0
    head_stiffness: float = 20.0
    head_damping: float = 2.0
    feedback_timeout_s: float = 2.0
    maximum_feedback_age_s: float = 0.25
    maximum_start_velocity_rad_s: float = 0.10
    maximum_start_error_rad: float = 0.20
    maximum_tracking_error_rad: float = 0.35
    maximum_temperature_c: int = 80
    final_hold_s: float = 0.30
    service_discovery_timeout_s: float = 15.0
    service_timeout_s: float = 0.50
    service_retries: int = 8
    mode_timeout_s: float = 5.0
    running_mode_status: int = 100
    stable_mode: str = "STAND_DEFAULT"
    split_mode: str = "UPPERBODY_REMOTE_SPLIT"
    command_source: str = "graspv2"
    command_source_priority: int = 65
    command_source_timeout_ms: int = 1000
    mode_source: str = "rc"


@dataclass(frozen=True)
class OmniPickerTuning:
    """Normalized OmniPicker command settings from the AimDK contract."""

    installed_side: str = INSTALLED_GRIPPER_SIDE
    right_joint_name: str = "right_claw_joint"
    require_feedback: bool = False
    open_position: float = 1.0
    closed_position: float = 0.0
    velocity: float = 1.0
    acceleration: float = 1.0
    deceleration: float = 1.0
    effort: float = 1.0
    publish_rate_hz: float = 50.0
    publish_duration_s: float = 2.0
    position_tolerance: float = 0.08


@dataclass(frozen=True)
class HardwareConfig:
    """Complete configuration for the live X2 AimDK boundary."""

    schema_version: int
    aimdk_api: str
    topics: AimDKTopics
    services: AimDKServices
    upper_body: UpperBodyTuning
    omnipicker: OmniPickerTuning


@dataclass(frozen=True)
class JointSetpoint:
    """One validated AimDK ``JointCommand`` independent of ROS types."""

    name: str
    position: float
    velocity: float
    effort: float
    stiffness: float
    damping: float


@dataclass(frozen=True)
class JointHealth:
    """Normalized health for both published X2 ``JointState`` layouts."""

    hottest_temperature_c: int | None
    error_codes: tuple[tuple[str, int], ...]
    packed_legacy_temperatures: bool = False


@dataclass(frozen=True)
class UpperBodyFrame:
    """One MC split-mode command in the official fixed-width layout."""

    head_positions: tuple[float, float]
    arm_positions: tuple[float, ...]
    hand_sub_mode: int = 0
    hand_positions: tuple[float, ...] = ()


class UpperBodyControl(Protocol):
    """Port implemented by an SDK-specific upper-body adapter."""

    def command_joint_group(
        self, group: str, commands: Sequence[JointSetpoint]
    ) -> None:
        ...

    def command_upper_body(self, frame: UpperBodyFrame) -> None:
        ...


class OmniPickerControl(Protocol):
    """Port implemented by an SDK-specific OmniPicker adapter."""

    def command_omnipicker(self, side: str, position: float) -> None:
        ...


def inspect_joint_health(joints: Sequence[object]) -> JointHealth:
    """Resolve native temperatures/errors and the X2 mixed-overlay layout.

    Some deployed X2 publishers still serialize the legacy ``coil_temp`` and
    ``motor_temp`` bytes while AimDK v1.0 exposes one ``uint16 error_code`` at
    the same offset.  DDS then presents ``(coil_temp << 8) | motor_temp`` as a
    non-zero error.  Detect that only at array level: at least four joints must
    carry plausible paired temperatures.  A lone non-zero value therefore
    remains a real fault and is never ignored.
    """

    if all(
        hasattr(joint, "coil_temp") and hasattr(joint, "motor_temp")
        for joint in joints
    ):
        temperatures = tuple(
            temperature
            for joint in joints
            for temperature in (
                int(getattr(joint, "coil_temp")),
                int(getattr(joint, "motor_temp")),
            )
        )
        return JointHealth(
            hottest_temperature_c=max(temperatures, default=None),
            error_codes=(),
        )

    named_codes = tuple(
        (
            str(getattr(joint, "name", "")) or "<unnamed>",
            int(getattr(joint, "error_code", 0)),
        )
        for joint in joints
    )
    nonzero = tuple(item for item in named_codes if item[1] != 0)
    decoded = tuple(
        (code >> 8, code & 0xFF)
        for _name, code in nonzero
    )
    if len(decoded) >= 4 and all(
        5 <= coil <= 125 and 5 <= motor <= 125
        for coil, motor in decoded
    ):
        return JointHealth(
            hottest_temperature_c=max(
                temperature
                for pair in decoded
                for temperature in pair
            ),
            error_codes=(),
            packed_legacy_temperatures=True,
        )
    return JointHealth(
        hottest_temperature_c=None,
        error_codes=nonzero,
    )


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise HardwareContractError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise HardwareContractError(f"{label} must be finite")
    return result


def _positive(value: object, label: str) -> float:
    result = _finite(value, label)
    if result <= 0.0:
        raise HardwareContractError(f"{label} must be positive")
    return result


def _object(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise HardwareContractError(f"{label} must be an object")
    return value


def _construct_dataclass(cls, values: Mapping[str, object], label: str):
    known = set(cls.__dataclass_fields__)
    extras = sorted(set(values) - known)
    if extras:
        raise HardwareContractError(
            f"{label} contains unknown fields: " + ", ".join(extras)
        )
    return cls(**values)


def _validate_config(config: HardwareConfig) -> HardwareConfig:
    if config.schema_version != 2:
        raise HardwareContractError("hardware schema_version must be 2")
    if not config.aimdk_api.strip():
        raise HardwareContractError("aimdk_api must be non-empty")
    for name, topic in vars(config.topics).items():
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise HardwareContractError(f"topics.{name} must be an absolute topic")
    for name, service in vars(config.services).items():
        if not isinstance(service, str) or not service.startswith("/"):
            raise HardwareContractError(
                f"services.{name} must be an absolute service"
            )

    upper = config.upper_body
    for name in (
        "command_rate_hz",
        "arm_stiffness",
        "arm_damping",
        "waist_stiffness",
        "waist_damping",
        "head_stiffness",
        "head_damping",
        "feedback_timeout_s",
        "maximum_feedback_age_s",
        "maximum_start_velocity_rad_s",
        "maximum_start_error_rad",
        "maximum_tracking_error_rad",
        "final_hold_s",
        "service_discovery_timeout_s",
        "service_timeout_s",
        "mode_timeout_s",
    ):
        _positive(getattr(upper, name), f"upper_body.{name}")
    if isinstance(upper.maximum_temperature_c, bool) or not isinstance(
        upper.maximum_temperature_c, int
    ):
        raise HardwareContractError(
            "upper_body.maximum_temperature_c must be an integer"
        )
    if upper.maximum_temperature_c <= 0:
        raise HardwareContractError(
            "upper_body.maximum_temperature_c must be positive"
        )
    for name in (
        "service_retries",
        "running_mode_status",
        "command_source_timeout_ms",
    ):
        value = getattr(upper, name)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise HardwareContractError(
                f"upper_body.{name} must be a positive integer"
            )
    priority = upper.command_source_priority
    if (
        isinstance(priority, bool)
        or not isinstance(priority, int)
        or not 0 <= priority <= 100
    ):
        raise HardwareContractError(
            "upper_body.command_source_priority must be an integer within [0, 100]"
        )
    for name in ("stable_mode", "split_mode", "command_source", "mode_source"):
        value = getattr(upper, name)
        if not isinstance(value, str) or not value.strip():
            raise HardwareContractError(f"upper_body.{name} must be non-empty")

    hand = config.omnipicker
    if hand.installed_side != INSTALLED_GRIPPER_SIDE:
        raise HardwareContractError(
            "omnipicker.installed_side must be 'right' for the competition robot"
        )
    if (
        not isinstance(hand.right_joint_name, str)
        or not hand.right_joint_name.strip()
    ):
        raise HardwareContractError("omnipicker.right_joint_name must be non-empty")
    if not isinstance(hand.require_feedback, bool):
        raise HardwareContractError(
            "omnipicker.require_feedback must be a boolean"
        )
    for name in (
        "open_position",
        "closed_position",
        "velocity",
        "acceleration",
        "deceleration",
        "effort",
        "publish_rate_hz",
        "publish_duration_s",
        "position_tolerance",
    ):
        value = _finite(getattr(hand, name), f"omnipicker.{name}")
        if name in {"open_position", "closed_position"}:
            if not 0.0 <= value <= 1.0:
                raise HardwareContractError(
                    f"omnipicker.{name} must be within [0, 1]"
                )
        elif value <= 0.0:
            raise HardwareContractError(f"omnipicker.{name} must be positive")
    return config


def require_installed_omnipicker_side(
    config: HardwareConfig,
    side: str,
) -> None:
    """Reject commands or plans for a side without a physical OmniPicker."""
    if side != config.omnipicker.installed_side:
        raise HardwareContractError(
            f"OmniPicker is installed only on {config.omnipicker.installed_side}; "
            f"refusing {side!r} gripper operation"
        )


def load_hardware_config(path: Path | None = None) -> HardwareConfig:
    """Load a strict JSON config, or use source/built-in defaults."""

    source = path.expanduser().resolve() if path is not None else None
    if source is None and DEFAULT_HARDWARE_CONFIG_PATH.is_file():
        source = DEFAULT_HARDWARE_CONFIG_PATH
    if source is None:
        return _validate_config(
            HardwareConfig(
                schema_version=2,
                aimdk_api="v0.9.0.7/v1.0 topic-compatible",
                topics=AimDKTopics(),
                services=AimDKServices(),
                upper_body=UpperBodyTuning(),
                omnipicker=OmniPickerTuning(),
            )
        )
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise HardwareContractError(
            f"hardware config does not exist: {source}"
        ) from error
    except json.JSONDecodeError as error:
        raise HardwareContractError(
            f"invalid hardware config at line {error.lineno}: {error.msg}"
        ) from error
    root = _object(document, "hardware config")
    expected = {
        "schema_version",
        "aimdk_api",
        "topics",
        "services",
        "upper_body",
        "omnipicker",
    }
    extras = sorted(set(root) - expected)
    missing = sorted(expected - set(root))
    if extras or missing:
        details = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if extras:
            details.append("unknown " + ", ".join(extras))
        raise HardwareContractError("hardware config: " + "; ".join(details))
    if isinstance(root["schema_version"], bool) or not isinstance(
        root["schema_version"], int
    ):
        raise HardwareContractError("schema_version must be an integer")
    if not isinstance(root["aimdk_api"], str):
        raise HardwareContractError("aimdk_api must be a string")
    config = HardwareConfig(
        schema_version=root["schema_version"],
        aimdk_api=root["aimdk_api"],
        topics=_construct_dataclass(
            AimDKTopics, _object(root["topics"], "topics"), "topics"
        ),
        services=_construct_dataclass(
            AimDKServices,
            _object(root["services"], "services"),
            "services",
        ),
        upper_body=_construct_dataclass(
            UpperBodyTuning,
            _object(root["upper_body"], "upper_body"),
            "upper_body",
        ),
        omnipicker=_construct_dataclass(
            OmniPickerTuning,
            _object(root["omnipicker"], "omnipicker"),
            "omnipicker",
        ),
    )
    return _validate_config(config)


def resolve_joint_feedback(
    joints: Sequence[object], expected_names: Sequence[str]
) -> tuple[float, ...]:
    """Return state positions in the verified hardware order.

    AimDK v0.8 documented the ``name`` field as unused.  Some v0.9 builds do
    populate it.  We support both cases but reject partial/mismatched names.
    """

    expected = tuple(expected_names)
    if len(joints) != len(expected):
        raise HardwareContractError(
            f"expected {len(expected)} joint states, received {len(joints)}"
        )
    names = tuple(str(getattr(joint, "name", "")) for joint in joints)
    if all(not name for name in names):
        ordered = joints
    elif all(names):
        if len(set(names)) != len(names):
            raise HardwareContractError("joint feedback contains duplicate names")
        by_name = dict(zip(names, joints))
        missing = sorted(set(expected) - set(by_name))
        extras = sorted(set(by_name) - set(expected))
        if missing or extras:
            raise HardwareContractError(
                "joint feedback names do not match the selected profile: "
                f"missing={missing}, unexpected={extras}"
            )
        ordered = tuple(by_name[name] for name in expected)
    else:
        raise HardwareContractError(
            "joint feedback has partially populated names; cannot prove order"
        )
    return tuple(
        _finite(getattr(joint, "position", None), f"state[{index}].position")
        for index, joint in enumerate(ordered)
    )


def arm_setpoints(
    profile: RobotProfile,
    base_positions: Sequence[float],
    active_positions: Mapping[str, float],
    active_velocities: Mapping[str, float] | None,
    tuning: UpperBodyTuning,
) -> tuple[JointSetpoint, ...]:
    """Overlay a planner frame onto live full-arm feedback."""

    expected = profile.arm_pos_order
    if len(base_positions) != len(expected):
        raise HardwareContractError(
            f"base arm position requires {len(expected)} values"
        )
    unsupported = sorted(set(active_positions) - set(expected))
    if unsupported:
        raise HardwareContractError(
            "active arm command contains unsupported joints: "
            + ", ".join(unsupported)
        )
    velocity_values = active_velocities or {}
    unsupported_velocities = sorted(set(velocity_values) - set(active_positions))
    if unsupported_velocities:
        raise HardwareContractError(
            "velocity supplied without an active position: "
            + ", ".join(unsupported_velocities)
        )
    result = []
    for name, base in zip(expected, base_positions):
        position = _finite(active_positions.get(name, base), f"{name}.position")
        velocity = _finite(velocity_values.get(name, 0.0), f"{name}.velocity")
        result.append(
            JointSetpoint(
                name=name,
                position=position,
                velocity=velocity,
                effort=0.0,
                stiffness=tuning.arm_stiffness,
                damping=tuning.arm_damping,
            )
        )
    return tuple(result)


def upper_body_arm_positions(
    profile: RobotProfile, positions: Sequence[float]
) -> tuple[float, ...]:
    """Validate an Ultra state in MC's fixed left-7/right-7 order."""

    if len(positions) != len(profile.arm_pos_order):
        raise HardwareContractError(
            f"profile arm position requires {len(profile.arm_pos_order)} values"
        )
    if profile.arm_pos_order != LEFT_ARM_7 + RIGHT_ARM_7:
        raise HardwareContractError("only the X2 Ultra 14-axis arm order is supported")
    return tuple(
        _finite(value, f"{name}.position")
        for name, value in zip(profile.arm_pos_order, positions)
    )


def trajectory_sample_velocity(
    trajectory: object, time_s: float, sample_period_s: float
) -> dict[str, float]:
    """Numerically differentiate any trajectory exposing ``sample``/``duration``."""

    if sample_period_s <= 0.0:
        raise HardwareContractError("sample_period_s must be positive")
    duration = _finite(getattr(trajectory, "duration", None), "trajectory.duration")
    before = max(0.0, time_s - sample_period_s)
    after = min(duration, time_s + sample_period_s)
    if after <= before:
        return {name: 0.0 for name in trajectory.sample(time_s)}
    first = trajectory.sample(before)
    second = trajectory.sample(after)
    if set(first) != set(second):
        raise HardwareContractError("trajectory sample joint names changed")
    return {
        name: (float(second[name]) - float(first[name])) / (after - before)
        for name in first
    }
