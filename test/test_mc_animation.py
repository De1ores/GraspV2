"""Tests for the MC animation conversion path."""

from pathlib import Path
from types import SimpleNamespace

import pytest

from graspv2.mc_animation import (
    DEFAULT_ARM_POSITION,
    MC_COLUMNS,
    build_mc_animation,
    build_mc_grasp_animation,
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
    assert info.grasp_hold_duration_s >= 0.09
    assert info.grasp_close_time_s < info.duration_s * 0.5


def test_default_animation_reserves_time_for_gripper_close(
    tmp_path: Path,
) -> None:
    animation = build_mc_animation(
        _trajectory(),
        speed_scale=0.5,
        lead_in_seconds=0.1,
        bridge_speed=0.25,
        maximum_output_velocity=0.6,
    )
    path = write_mc_animation_csv(animation, tmp_path / "grasp.csv")
    info = validate_mc_animation_csv(path, maximum_velocity=0.6)

    assert info.grasp_hold_duration_s >= 1.99
    assert info.grasp_close_time_s + info.grasp_hold_duration_s <= (
        info.duration_s - info.grasp_close_time_s + 0.01
    )


def test_complete_grasp_animation_matches_all_simulated_phases(
    tmp_path: Path,
) -> None:
    joint = "right_shoulder_pitch_joint"

    def trajectory(name, times, positions):
        return JointTrajectory(
            source=Path(name),
            joint_names=(joint,),
            times=times,
            positions=tuple((value,) for value in positions),
            maximum_velocity=3.0,
        )

    approach = trajectory("approach.json", (0.0, 0.04, 0.1), (0.4, 0.35, 0.3))
    lift = trajectory("lift.json", (0.0, 0.05), (0.3, 0.2))
    return_path = trajectory(
        "return.json",
        (0.0, 0.04, 0.07, 0.1),
        (0.2, 0.25, 0.3, 0.4),
    )
    metadata = SimpleNamespace(
        pregrasp_duration_s=0.04,
        controlled_lower_duration_s=0.04,
        open_hand_retreat_duration_s=0.03,
        open_duration_s=0.02,
        close_duration_s=0.02,
        grasp_settle_duration_s=0.02,
        lifted_hold_duration_s=0.02,
        release_duration_s=0.02,
        place_settle_duration_s=0.02,
        reclose_duration_s=0.02,
        preopen_position=1.0,
        grip_position=0.55,
    )

    animation = build_mc_grasp_animation(
        approach,
        lift,
        return_path,
        metadata,
        speed_scale=1.0,
        maximum_output_velocity=4.0,
    )

    assert animation.bridge_duration_s == 0.0
    assert animation.duration_s == pytest.approx(0.39)
    assert animation.initial_gripper_position == 0.0
    assert [event.label for event in animation.gripper_events] == [
        "fully-open-at-pregrasp",
        "close-to-visual-radius",
        "fully-open-placed-release",
        "close-empty-at-pregrasp",
    ]
    assert [event.time_s for event in animation.gripper_events] == pytest.approx(
        [0.04, 0.12, 0.27, 0.34]
    )
    assert [event.position for event in animation.gripper_events] == pytest.approx(
        [1.0, 0.55, 1.0, 0.0]
    )
    assert tuple(animation.rows[-1][4:18]) == pytest.approx(
        DEFAULT_ARM_POSITION
    )
    path = write_mc_animation_csv(animation, tmp_path / "full-grasp.csv")
    info = validate_mc_animation_csv(path, maximum_velocity=4.0)
    assert info.duration_s == pytest.approx(animation.duration_s)


def test_csv_validation_rejects_wrong_header(tmp_path: Path) -> None:
    path = tmp_path / "bad.csv"
    path.write_text(",".join(MC_COLUMNS[:-1]) + "\n", encoding="utf-8")
    with pytest.raises(TrajectoryValidationError, match="20-column"):
        validate_mc_animation_csv(path)
