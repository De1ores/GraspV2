"""Validation and interpolation for X2 active-joint JSON trajectories."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Sequence


ARM_JOINT_ORDER = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
)

ARM_JOINT_INDEX = {
    name: index for index, name in enumerate(ARM_JOINT_ORDER)
}

# Limits from the X2 AimDK joint-control example. These checks are deliberately
# duplicated here so an unsafe JSON file is rejected before ROS is initialized.
ARM_JOINT_LIMITS = {
    "left_shoulder_pitch_joint": (-3.08, 2.04),
    "left_shoulder_roll_joint": (-0.061, 2.993),
    "left_shoulder_yaw_joint": (-2.556, 2.556),
    "left_elbow_joint": (-2.3556, 0.0),
    "left_wrist_yaw_joint": (-2.556, 2.556),
    "left_wrist_pitch_joint": (-0.558, 0.558),
    "left_wrist_roll_joint": (-1.571, 0.724),
    "right_shoulder_pitch_joint": (-3.08, 2.04),
    "right_shoulder_roll_joint": (-2.993, 0.061),
    "right_shoulder_yaw_joint": (-2.556, 2.556),
    "right_elbow_joint": (-2.3556, 0.0),
    "right_wrist_yaw_joint": (-2.556, 2.556),
    "right_wrist_pitch_joint": (-0.558, 0.558),
    "right_wrist_roll_joint": (-0.724, 1.571),
}


class TrajectoryValidationError(ValueError):
    """Raised when a trajectory cannot safely be sent to the robot."""


def _finite_float(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TrajectoryValidationError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise TrajectoryValidationError(f"{label} must be finite")
    return result


@dataclass(frozen=True)
class JointTrajectory:
    """A validated joint trajectory with positions aligned to ``joint_names``."""

    source: Path
    joint_names: tuple[str, ...]
    times: tuple[float, ...]
    positions: tuple[tuple[float, ...], ...]
    maximum_velocity: float

    @property
    def duration(self) -> float:
        return self.times[-1]

    @property
    def frame_count(self) -> int:
        return len(self.times)

    def sample(self, time_s: float) -> dict[str, float]:
        """Linearly resample the trajectory at an arbitrary source timestamp."""
        if time_s <= self.times[0]:
            values = self.positions[0]
        elif time_s >= self.times[-1]:
            values = self.positions[-1]
        else:
            upper = bisect_right(self.times, time_s)
            lower = upper - 1
            start_time = self.times[lower]
            duration = self.times[upper] - start_time
            ratio = (time_s - start_time) / duration
            values = tuple(
                start + ratio * (end - start)
                for start, end in zip(
                    self.positions[lower],
                    self.positions[upper],
                )
            )
        return dict(zip(self.joint_names, values))

    def apply(
        self,
        base_arm_positions: Sequence[float],
        time_s: float,
    ) -> list[float]:
        """Overlay a sampled active-joint frame onto a complete 14-DOF arm pose."""
        if len(base_arm_positions) != len(ARM_JOINT_ORDER):
            raise ValueError("base_arm_positions must contain exactly 14 values")
        result = [float(value) for value in base_arm_positions]
        for name, position in self.sample(time_s).items():
            result[ARM_JOINT_INDEX[name]] = position
        return result


def reverse_trajectory(
    trajectory: JointTrajectory,
    *,
    source: Path | None = None,
) -> JointTrajectory:
    """Return the same samples in reverse with time starting again at zero."""

    duration = trajectory.duration
    times = tuple(
        round(duration - original, 9)
        for original in reversed(trajectory.times)
    )
    # Keep exact zero/duration endpoints despite decimal input roundoff.
    times = (0.0, *times[1:-1], duration)
    return JointTrajectory(
        source=source or trajectory.source,
        joint_names=trajectory.joint_names,
        times=times,
        positions=tuple(reversed(trajectory.positions)),
        maximum_velocity=trajectory.maximum_velocity,
    )


def load_trajectory(
    path: Path,
    maximum_allowed_velocity: float = 1.5,
) -> JointTrajectory:
    """Load the repository's ``x2_active_joint_trajectory`` JSON format."""
    source = path.expanduser().resolve()
    if maximum_allowed_velocity <= 0.0:
        raise ValueError("maximum_allowed_velocity must be positive")
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise TrajectoryValidationError(
            f"trajectory file does not exist: {source}"
        ) from error
    except json.JSONDecodeError as error:
        raise TrajectoryValidationError(
            f"invalid trajectory JSON at line {error.lineno}: {error.msg}"
        ) from error

    if not isinstance(document, dict):
        raise TrajectoryValidationError("trajectory root must be an object")
    if document.get("format") != "x2_active_joint_trajectory":
        raise TrajectoryValidationError(
            "format must be 'x2_active_joint_trajectory'"
        )
    units = document.get("units")
    if not isinstance(units, dict):
        raise TrajectoryValidationError("units must be an object")
    if units.get("time") != "s" or units.get("joint_angle") != "rad":
        raise TrajectoryValidationError(
            "trajectory units must be seconds and radians"
        )

    raw_names = document.get("active_joint_names")
    if not isinstance(raw_names, list) or not raw_names:
        raise TrajectoryValidationError(
            "active_joint_names must be a non-empty array"
        )
    if not all(isinstance(name, str) and name for name in raw_names):
        raise TrajectoryValidationError(
            "active_joint_names must contain non-empty strings"
        )
    joint_names = tuple(raw_names)
    if len(set(joint_names)) != len(joint_names):
        raise TrajectoryValidationError("active_joint_names contains duplicates")
    unsupported = sorted(set(joint_names) - set(ARM_JOINT_ORDER))
    if unsupported:
        raise TrajectoryValidationError(
            "trajectory contains unsupported joints: " + ", ".join(unsupported)
        )

    raw_frames = document.get("frames")
    if not isinstance(raw_frames, list) or len(raw_frames) < 2:
        raise TrajectoryValidationError(
            "frames must contain at least two trajectory frames"
        )
    declared_count = document.get("frame_count")
    if declared_count != len(raw_frames):
        raise TrajectoryValidationError(
            f"frame_count is {declared_count!r}, but {len(raw_frames)} frames exist"
        )

    times: list[float] = []
    positions: list[tuple[float, ...]] = []
    for index, raw_frame in enumerate(raw_frames):
        label = f"frames[{index}]"
        if not isinstance(raw_frame, dict):
            raise TrajectoryValidationError(f"{label} must be an object")
        if raw_frame.get("frame") != index:
            raise TrajectoryValidationError(
                f"{label}.frame must be the sequential index {index}"
            )
        time_s = _finite_float(raw_frame.get("time_s"), f"{label}.time_s")
        if time_s < 0.0:
            raise TrajectoryValidationError(f"{label}.time_s must be non-negative")
        if times and time_s <= times[-1]:
            raise TrajectoryValidationError(
                f"{label}.time_s must be strictly increasing"
            )

        raw_positions = raw_frame.get("active_joints_rad")
        if not isinstance(raw_positions, dict):
            raise TrajectoryValidationError(
                f"{label}.active_joints_rad must be an object"
            )
        missing = [name for name in joint_names if name not in raw_positions]
        extras = sorted(set(raw_positions) - set(joint_names))
        if missing or extras:
            details = []
            if missing:
                details.append("missing " + ", ".join(missing))
            if extras:
                details.append("unexpected " + ", ".join(extras))
            raise TrajectoryValidationError(
                f"{label}.active_joints_rad: " + "; ".join(details)
            )

        frame_positions = []
        for name in joint_names:
            position = _finite_float(
                raw_positions[name],
                f"{label}.active_joints_rad[{name!r}]",
            )
            lower, upper = ARM_JOINT_LIMITS[name]
            if position < lower or position > upper:
                raise TrajectoryValidationError(
                    f"{label} joint {name}={position:.6f} rad is outside "
                    f"[{lower:.6f}, {upper:.6f}]"
                )
            frame_positions.append(position)
        times.append(time_s)
        positions.append(tuple(frame_positions))

    if abs(times[0]) > 1e-9:
        raise TrajectoryValidationError("the first frame must have time_s=0")
    declared_duration = _finite_float(
        document.get("duration_s"),
        "duration_s",
    )
    if abs(declared_duration - times[-1]) > 1e-6:
        raise TrajectoryValidationError(
            f"duration_s={declared_duration} does not match last frame "
            f"time_s={times[-1]}"
        )

    maximum_velocity = 0.0
    maximum_velocity_location = ""
    for frame_index in range(1, len(times)):
        delta_time = times[frame_index] - times[frame_index - 1]
        for joint_index, name in enumerate(joint_names):
            velocity = abs(
                positions[frame_index][joint_index]
                - positions[frame_index - 1][joint_index]
            ) / delta_time
            if velocity > maximum_velocity:
                maximum_velocity = velocity
                maximum_velocity_location = (
                    f"{name} between frames {frame_index - 1} and {frame_index}"
                )
    if maximum_velocity > maximum_allowed_velocity:
        raise TrajectoryValidationError(
            f"trajectory speed {maximum_velocity:.3f} rad/s at "
            f"{maximum_velocity_location} exceeds the configured "
            f"{maximum_allowed_velocity:.3f} rad/s limit"
        )

    return JointTrajectory(
        source=source,
        joint_names=joint_names,
        times=tuple(times),
        positions=tuple(positions),
        maximum_velocity=maximum_velocity,
    )
