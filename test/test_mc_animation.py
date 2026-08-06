"""Tests for the MC animation conversion path."""

from pathlib import Path

import pytest

from graspv2.mc_animation import (
    DEFAULT_ARM_POSITION,
    MC_COLUMNS,
    build_mc_animation,
    validate_mc_animation_csv,
    write_mc_animation_csv,
)
from graspv2.trajectory import JointTrajectory, TrajectoryValidationError


def _trajectory() -> JointTrajectory:
    return JointTrajectory(
        source=Path("synthetic.json"),
        joint_names=("right_shoulder_pitch_joint",),
        times=(0.0, 0.1),
        positions=((0.4,), (0.3,)),
        maximum_velocity=1.0,
    )


def test_animation_contains_reverse_return_and_default_endpoint() -> None:
    animation = build_mc_animation(
        _trajectory(),
        speed_scale=0.5,
        hold_seconds=0.1,
        lead_in_seconds=0.1,
        bridge_speed=0.25,
        maximum_output_velocity=0.6,
    )
    right_pitch_column = 1 + 3 + 7
    values = [row[right_pitch_column] for row in animation.rows]
    assert min(values) == pytest.approx(0.3)
    assert values[0] == pytest.approx(0.4)
    assert values[-1] == pytest.approx(0.4)
    assert tuple(animation.rows[-1][4:18]) == pytest.approx(
        DEFAULT_ARM_POSITION
    )
    assert animation.return_path_enabled
    assert len(MC_COLUMNS) == 20
    assert all("thumb" not in column for column in MC_COLUMNS)


def test_csv_round_trip_validation(tmp_path: Path) -> None:
    animation = build_mc_animation(
        _trajectory(),
        speed_scale=0.5,
        hold_seconds=0.1,
        lead_in_seconds=0.1,
        bridge_speed=0.25,
        maximum_output_velocity=0.6,
    )
    path = write_mc_animation_csv(animation, tmp_path / "grasp.csv")
    info = validate_mc_animation_csv(path, maximum_velocity=0.6)
    assert info.frame_count == animation.frame_count
    assert info.duration_s == pytest.approx(animation.duration_s)


def test_csv_validation_rejects_wrong_header(tmp_path: Path) -> None:
    path = tmp_path / "bad.csv"
    path.write_text(",".join(MC_COLUMNS[:-1]) + "\n", encoding="utf-8")
    with pytest.raises(TrajectoryValidationError, match="20-column"):
        validate_mc_animation_csv(path)
