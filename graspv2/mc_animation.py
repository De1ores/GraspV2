"""Convert a verified grasp trajectory to the X2 MC animation CSV format."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, Sequence

from .trajectory import (
    ARM_JOINT_LIMITS,
    ARM_JOINT_ORDER,
    JointTrajectory,
    TrajectoryValidationError,
)
from .robot_profiles import MC_DEFAULT_ARM_POSITION


DEFAULT_ANIMATION_NAME = "output/mc_animation.csv"
MC_SAMPLE_PERIOD_S = 0.002

WAIST_JOINTS = (
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
)
HEAD_JOINTS = ("head_yaw_joint", "head_pitch_joint")
# The robot has no dexterous hands attached. Existing MC resources demonstrate
# that hand columns are optional, so deliberately omit them instead of sending
# commands for hardware that is not present.
MC_JOINT_ORDER = WAIST_JOINTS + ARM_JOINT_ORDER + HEAD_JOINTS
MC_COLUMNS = ("timeMS",) + tuple(
    f"command_pos::{name}" for name in MC_JOINT_ORDER
)

# These are the MC animation player's own classic standing values. The player
# plans from the live pose to this first row before replaying the CSV.
DEFAULT_ARM_POSITION = MC_DEFAULT_ARM_POSITION


@dataclass(frozen=True)
class McAnimation:
    """Validated, fully sampled MC animation data."""

    rows: tuple[tuple[float, ...], ...]
    duration_s: float
    bridge_duration_s: float
    maximum_arm_velocity: float
    return_path_enabled: bool = True

    @property
    def frame_count(self) -> int:
        return len(self.rows)


@dataclass(frozen=True)
class McCsvInfo:
    """Summary returned after parsing a generated MC CSV."""

    frame_count: int
    duration_s: float
    maximum_arm_velocity: float


def _minimum_jerk(ratio: float) -> float:
    ratio = max(0.0, min(1.0, ratio))
    return ratio**3 * (10.0 + ratio * (-15.0 + 6.0 * ratio))


def _interpolate(
    start: Sequence[float],
    end: Sequence[float],
    ratio: float,
) -> list[float]:
    blend = _minimum_jerk(ratio)
    return [
        float(first + blend * (second - first))
        for first, second in zip(start, end)
    ]


def _aligned_duration(duration_s: float) -> float:
    if duration_s <= 0.0:
        return 0.0
    ticks = max(1, math.ceil(duration_s / MC_SAMPLE_PERIOD_S - 1e-12))
    return ticks * MC_SAMPLE_PERIOD_S


def _phase_times(duration_s: float) -> Iterable[float]:
    """Yield 2 ms samples after zero, including the aligned phase endpoint."""
    ticks = int(round(duration_s / MC_SAMPLE_PERIOD_S))
    for tick in range(1, ticks + 1):
        yield tick * MC_SAMPLE_PERIOD_S


def _row(time_s: float, arm_position: Sequence[float]) -> tuple[float, ...]:
    if len(arm_position) != len(ARM_JOINT_ORDER):
        raise ValueError("MC animation arm position must contain 14 values")
    return (
        time_s * 1000.0,
        0.0,
        0.0,
        0.0,
        *[float(value) for value in arm_position],
        0.0,
        0.0,
    )


def _maximum_arm_velocity(rows: Sequence[Sequence[float]]) -> float:
    maximum = 0.0
    arm_start = 1 + len(WAIST_JOINTS)
    arm_stop = arm_start + len(ARM_JOINT_ORDER)
    for previous, current in zip(rows, rows[1:]):
        delta_s = (float(current[0]) - float(previous[0])) / 1000.0
        if delta_s <= 0.0:
            raise TrajectoryValidationError(
                "MC animation timestamps must be strictly increasing"
            )
        for before, after in zip(
            previous[arm_start:arm_stop],
            current[arm_start:arm_stop],
        ):
            maximum = max(maximum, abs(float(after) - float(before)) / delta_s)
    return maximum


def build_mc_animation(
    trajectory: JointTrajectory,
    *,
    speed_scale: float = 0.5,
    hold_seconds: float = 0.5,
    lead_in_seconds: float = 1.0,
    bridge_speed: float = 0.25,
    maximum_output_velocity: float = 0.5,
) -> McAnimation:
    """Build one MC-owned approach, grasp hold and reverse-return animation."""
    if not 0.0 < speed_scale <= 1.0:
        raise ValueError("speed_scale must be in (0, 1]")
    if hold_seconds < 0.0:
        raise ValueError("hold_seconds must be non-negative")
    if lead_in_seconds <= 0.0:
        raise ValueError("lead_in_seconds must be positive")
    if bridge_speed <= 0.0:
        raise ValueError("bridge_speed must be positive")
    if maximum_output_velocity <= 0.0:
        raise ValueError("maximum_output_velocity must be positive")

    default_arm = list(DEFAULT_ARM_POSITION)
    first_arm = trajectory.apply(default_arm, 0.0)
    final_arm = trajectory.apply(default_arm, trajectory.duration)
    maximum_bridge_delta = max(
        abs(target - current)
        for target, current in zip(first_arm, default_arm)
    )
    # Minimum-jerk peak speed is 1.875 times delta/duration.
    bridge_duration = _aligned_duration(
        max(
            lead_in_seconds,
            1.875 * maximum_bridge_delta / bridge_speed,
        )
    )
    reach_duration = _aligned_duration(trajectory.duration / speed_scale)
    hold_duration = _aligned_duration(hold_seconds)

    rows: list[tuple[float, ...]] = [_row(0.0, default_arm)]
    elapsed = 0.0

    for phase_time in _phase_times(bridge_duration):
        rows.append(
            _row(
                elapsed + phase_time,
                _interpolate(
                    default_arm,
                    first_arm,
                    phase_time / bridge_duration,
                ),
            )
        )
    elapsed += bridge_duration

    for phase_time in _phase_times(reach_duration):
        source_time = min(
            trajectory.duration,
            phase_time * trajectory.duration / reach_duration,
        )
        position = trajectory.apply(default_arm, source_time)
        rows.append(_row(elapsed + phase_time, position))
    elapsed += reach_duration

    for phase_time in _phase_times(hold_duration):
        rows.append(_row(elapsed + phase_time, final_arm))
    elapsed += hold_duration

    # The reverse segment is part of the CSV itself. Once MC accepts the
    # animation, retreat no longer depends on this ROS process remaining alive.
    for phase_time in _phase_times(reach_duration):
        source_time = max(
            0.0,
            trajectory.duration
            * (1.0 - phase_time / reach_duration),
        )
        position = trajectory.apply(default_arm, source_time)
        rows.append(_row(elapsed + phase_time, position))
    elapsed += reach_duration

    for phase_time in _phase_times(bridge_duration):
        rows.append(
            _row(
                elapsed + phase_time,
                _interpolate(
                    first_arm,
                    default_arm,
                    phase_time / bridge_duration,
                ),
            )
        )
    elapsed += bridge_duration

    maximum_velocity = _maximum_arm_velocity(rows)
    if maximum_velocity > maximum_output_velocity + 1e-6:
        raise TrajectoryValidationError(
            f"generated MC animation reaches {maximum_velocity:.3f} rad/s, "
            f"above the configured {maximum_output_velocity:.3f} rad/s limit"
        )
    if any(abs(a - b) > 1e-9 for a, b in zip(rows[-1][4:18], default_arm)):
        raise AssertionError(
            "generated MC animation does not end at default pose"
        )

    return McAnimation(
        rows=tuple(rows),
        duration_s=elapsed,
        bridge_duration_s=bridge_duration,
        maximum_arm_velocity=maximum_velocity,
    )


def write_mc_animation_csv(animation: McAnimation, path: Path) -> Path:
    """Write atomically so the MC player never sees a partial CSV."""
    destination = path.expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(MC_COLUMNS)
        for row in animation.rows:
            positions = [f"{value:.9f}" for value in row[1:]]
            writer.writerow([f"{row[0]:.1f}", *positions])
    temporary.replace(destination)
    return destination


def validate_mc_animation_csv(
    path: Path,
    *,
    maximum_velocity: float = 0.5,
) -> McCsvInfo:
    """Validate a generated CSV before it is copied to or requested by MC."""
    source = path.expanduser().resolve()
    try:
        stream = source.open("r", encoding="utf-8", newline="")
    except FileNotFoundError as error:
        raise TrajectoryValidationError(
            f"MC animation CSV does not exist: {source}"
        ) from error

    rows: list[tuple[float, ...]] = []
    with stream:
        reader = csv.reader(stream)
        try:
            header = tuple(next(reader))
        except StopIteration as error:
            raise TrajectoryValidationError(
                "MC animation CSV is empty"
            ) from error
        if header != MC_COLUMNS:
            raise TrajectoryValidationError(
                "MC animation CSV header does not match the 20-column "
                "hand-free animation_player format"
            )
        for line_number, raw_row in enumerate(reader, start=2):
            if len(raw_row) != len(MC_COLUMNS):
                raise TrajectoryValidationError(
                    f"MC animation CSV line {line_number} has {len(raw_row)} "
                    f"columns; expected {len(MC_COLUMNS)}"
                )
            try:
                values = tuple(float(value) for value in raw_row)
            except ValueError as error:
                raise TrajectoryValidationError(
                    f"MC animation CSV line {line_number} contains "
                    "non-numeric data"
                ) from error
            if not all(math.isfinite(value) for value in values):
                raise TrajectoryValidationError(
                    f"MC animation CSV line {line_number} contains "
                    "non-finite data"
                )
            rows.append(values)

    if len(rows) < 2:
        raise TrajectoryValidationError(
            "MC animation CSV must contain at least two frames"
        )
    if abs(rows[0][0]) > 1e-9:
        raise TrajectoryValidationError(
            "MC animation CSV must start at timeMS=0"
        )
    arm_start = 1 + len(WAIST_JOINTS)
    for line_number, row in enumerate(rows, start=2):
        for index, name in enumerate(ARM_JOINT_ORDER):
            position = row[arm_start + index]
            lower, upper = ARM_JOINT_LIMITS[name]
            if position < lower or position > upper:
                raise TrajectoryValidationError(
                    f"MC animation CSV line {line_number}: "
                    f"{name}={position:.6f} "
                    f"is outside [{lower:.6f}, {upper:.6f}]"
                )
    measured_velocity = _maximum_arm_velocity(rows)
    if measured_velocity > maximum_velocity + 1e-6:
        raise TrajectoryValidationError(
            f"MC animation CSV reaches {measured_velocity:.3f} rad/s, above "
            f"the configured {maximum_velocity:.3f} rad/s limit"
        )
    final_arm = rows[-1][arm_start:arm_start + len(ARM_JOINT_ORDER)]
    if any(abs(a - b) > 1e-6 for a, b in zip(final_arm, DEFAULT_ARM_POSITION)):
        raise TrajectoryValidationError(
            "MC animation CSV does not finish at the animation-player "
            "default pose"
        )
    return McCsvInfo(
        frame_count=len(rows),
        duration_s=rows[-1][0] / 1000.0,
        maximum_arm_velocity=measured_velocity,
    )
