"""Tests for the visual grasp/no-drop contract."""

import json
from pathlib import Path

import pytest

from graspv2.grasp_sequence import (
    GraspPlanMetadata,
    VisualObservation,
    VisualVerificationConfig,
    VisualVerificationError,
    load_grasp_plan_metadata,
    load_visual_observation,
    validate_trajectory_continuity,
    verify_closed_observation,
    verify_lifted_observation,
)
from graspv2.trajectory import (
    JointTrajectory,
    TrajectoryValidationError,
    reverse_trajectory,
    slice_trajectory,
)


def _observation(path: Path, point: tuple[float, float, float]) -> VisualObservation:
    return VisualObservation(
        source=path,
        class_name="bottle",
        confidence=0.9,
        object_center_world_m=point,
    )


def _trajectory(path: Path, start: float, end: float) -> JointTrajectory:
    return JointTrajectory(
        source=path,
        joint_names=("right_shoulder_pitch_joint",),
        times=(0.0, 2.0),
        positions=((start,), (end,)),
        maximum_velocity=abs(end - start) / 2.0,
    )


def test_visual_gates_require_object_to_follow_two_second_lift(tmp_path: Path) -> None:
    settings = VisualVerificationConfig()
    metadata = GraspPlanMetadata(
        robot_profile="ultra",
        side="right",
        object_center_world_m=(0.40, -0.20, 0.80),
        gripper_center_world_m=(0.40, -0.20, 0.805),
        lifted_object_center_world_m=(0.40, -0.20, 0.90),
        lifted_gripper_center_world_m=(0.40, -0.20, 0.905),
        lift_direction_world=(0.0, 0.0, 1.0),
        lift_height_m=0.10,
        lift_duration_s=2.0,
        pregrasp_duration_s=1.0,
        visual_radius_m=0.045,
        preopen_position=1.0,
        grip_position=0.72,
        open_duration_s=0.5,
        close_duration_s=0.8,
        grasp_settle_duration_s=0.3,
        lifted_hold_duration_s=2.5,
        controlled_lower_duration_s=2.0,
        open_hand_retreat_duration_s=0.5,
        release_duration_s=0.6,
        place_settle_duration_s=0.7,
        reclose_duration_s=0.6,
    )
    closed = _observation(tmp_path / "closed.json", (0.405, -0.20, 0.802))
    closed_result = verify_closed_observation(
        closed,
        metadata.object_center_world_m,
        settings,
    )
    assert closed_result.passed

    lifted = _observation(tmp_path / "lifted.json", (0.405, -0.20, 0.897))
    lifted_result = verify_lifted_observation(
        closed,
        lifted,
        metadata,
        settings,
    )
    assert lifted_result.lift_displacement_m == pytest.approx(0.095)

    dropped = _observation(tmp_path / "dropped.json", (0.405, -0.20, 0.805))
    with pytest.raises(VisualVerificationError, match="dropped or did not follow"):
        verify_lifted_observation(closed, dropped, metadata, settings)


def test_closed_gate_rejects_wrong_grasp_region(tmp_path: Path) -> None:
    observation = _observation(tmp_path / "closed.json", (0.60, 0.10, 0.80))
    with pytest.raises(VisualVerificationError, match="closed-grasp"):
        verify_closed_observation(
            observation,
            (0.40, -0.20, 0.80),
            VisualVerificationConfig(),
        )


def test_vision_loader_requires_requested_class(tmp_path: Path) -> None:
    path = tmp_path / "result.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 2,
                "surface_point_mujoco_m": [0.4, -0.2, 0.82],
                "object_center_mujoco_m": [0.4, -0.2, 0.8],
                "gripper_center_mujoco_m": [0.4, -0.2, 0.805],
                "selected_detection": {
                    "class_name": "cup",
                    "confidence": 0.8,
                },
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(VisualVerificationError, match="requested 'bottle'"):
        load_visual_observation(path, "bottle")


def test_vision_loader_tracks_object_center_not_surface_point(tmp_path: Path) -> None:
    path = tmp_path / "result.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 2,
                "surface_point_mujoco_m": [0.36, -0.25, 0.94],
                "object_center_mujoco_m": [0.38, -0.25, 0.86],
                "gripper_center_mujoco_m": [0.38, -0.25, 0.865],
                "selected_detection": {
                    "class_name": "cup",
                    "confidence": 0.8,
                },
            }
        ),
        encoding="utf-8",
    )

    observation = load_visual_observation(path, "cup")
    assert observation.object_center_world_m == pytest.approx((0.38, -0.25, 0.86))
    assert observation.surface_point_world_m == pytest.approx((0.36, -0.25, 0.94))


def test_vision_loader_rejects_legacy_surface_as_object_center(tmp_path: Path) -> None:
    path = tmp_path / "legacy_result.json"
    path.write_text(
        json.dumps(
            {
                "grasp_point_mujoco_m": [0.36, -0.25, 0.94],
                "selected_detection": {
                    "class_name": "cup",
                    "confidence": 0.8,
                },
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(VisualVerificationError, match="legacy surface point"):
        load_visual_observation(path, "cup")


def test_grasp_plan_metadata_and_joint_continuity(tmp_path: Path) -> None:
    common = {
        "format": "x2_active_joint_trajectory",
        "robot_profile": "ultra",
        "arm_side": "right",
    }
    approach_path = tmp_path / "approach.json"
    approach_path.write_text(
        json.dumps(
            {
                **common,
                "planning": {
                    "verified_collision_free": True,
                    "ik_backend": "x2_ik_sdk.X2ArmIKSolver",
                    "trajectory_role": "approach",
                    "object_center_world_m": [0.4, -0.2, 0.795],
                    "gripper_center_world_m": [0.4, -0.2, 0.8],
                    "target_world_m": [0.4, -0.2, 0.8],
                    "vertical_descent_start_time_s": 1.0,
                    "simulated_grasp_sequence": {
                        "visual_radius_m": 0.045,
                        "preopen_position": 1.0,
                        "grip_position": 0.72,
                        "vertical_descent_start_time_s": 1.0,
                        "open_duration_s": 0.5,
                        "close_duration_s": 0.8,
                        "grasp_settle_duration_s": 0.3,
                        "lifted_hold_duration_s": 2.5,
                        "controlled_lower_duration_s": 2.0,
                        "open_hand_retreat_duration_s": 0.5,
                        "release_duration_s": 0.6,
                        "place_settle_duration_s": 0.7,
                        "reclose_duration_s": 0.6,
                    },
                },
            }
        ),
        encoding="utf-8",
    )
    lift_path = tmp_path / "lift.json"
    lift_path.write_text(
        json.dumps(
            {
                **common,
                "planning": {
                    "verified_collision_free": True,
                    "ik_backend": "x2_ik_sdk.X2ArmIKSolver",
                    "trajectory_role": "lift",
                    "lift_start_object_center_world_m": [0.4, -0.2, 0.795],
                    "object_center_world_m": [0.4, -0.2, 0.895],
                    "lift_start_gripper_center_world_m": [0.4, -0.2, 0.8],
                    "gripper_center_world_m": [0.4, -0.2, 0.9],
                    "lift_start_world_m": [0.4, -0.2, 0.8],
                    "target_world_m": [0.4, -0.2, 0.9],
                    "lift_direction_world": [0.0, 0.0, 1.0],
                    "lift_height_m": 0.1,
                    "lift_duration_s": 2.0,
                },
            }
        ),
        encoding="utf-8",
    )
    return_path = tmp_path / "return.json"
    return_path.write_text(
        json.dumps(
            {
                **common,
                "planning": {
                    "verified_collision_free": True,
                    "ik_backend": "x2_ik_sdk.X2ArmIKSolver",
                    "trajectory_role": "return_to_default",
                    "return_mode": "controlled_lower_then_reverse_approach",
                    "controlled_lower_duration_s": 2.0,
                    "open_hand_retreat_duration_s": 0.5,
                },
            }
        ),
        encoding="utf-8",
    )
    metadata = load_grasp_plan_metadata(approach_path, lift_path, return_path)
    assert metadata.lift_duration_s == pytest.approx(2.0)
    assert metadata.preopen_position == pytest.approx(1.0)
    assert metadata.pregrasp_duration_s == pytest.approx(1.0)
    assert metadata.object_center_world_m == pytest.approx((0.4, -0.2, 0.795))
    assert metadata.gripper_center_world_m == pytest.approx((0.4, -0.2, 0.8))

    lift_document = json.loads(lift_path.read_text(encoding="utf-8"))
    lift_document["planning"].update(
        {
            "actual_lift_displacement_world_m": [0.01, 0.0, 0.1],
            "gripper_center_world_m": [0.41, -0.2, 0.9],
            "target_world_m": [0.41, -0.2, 0.9],
            "object_center_world_m": [0.41, -0.2, 0.895],
        }
    )
    lift_path.write_text(json.dumps(lift_document), encoding="utf-8")
    nearest_metadata = load_grasp_plan_metadata(
        approach_path, lift_path, return_path
    )
    assert nearest_metadata.lifted_gripper_center_world_m == pytest.approx(
        (0.41, -0.2, 0.9)
    )
    assert nearest_metadata.lifted_object_center_world_m == pytest.approx(
        (0.41, -0.2, 0.895)
    )

    approach = _trajectory(approach_path, 0.0, 0.2)
    lift = _trajectory(lift_path, 0.2, 0.3)
    validate_trajectory_continuity(approach, lift)
    with pytest.raises(TrajectoryValidationError, match="discontinuity"):
        validate_trajectory_continuity(approach, _trajectory(lift_path, 0.1, 0.3))


def test_reverse_trajectory_preserves_duration_and_endpoints(tmp_path: Path) -> None:
    trajectory = _trajectory(tmp_path / "forward.json", 0.1, 0.4)
    reversed_trajectory = reverse_trajectory(trajectory)
    assert reversed_trajectory.times == (0.0, 2.0)
    assert reversed_trajectory.positions == ((0.4,), (0.1,))
    assert reversed_trajectory.maximum_velocity == trajectory.maximum_velocity


def test_slice_trajectory_rebases_interpolated_boundaries(tmp_path: Path) -> None:
    trajectory = JointTrajectory(
        source=tmp_path / "whole.json",
        joint_names=("right_shoulder_pitch_joint",),
        times=(0.0, 1.0, 2.0),
        positions=((0.0,), (0.2,), (0.6,)),
        maximum_velocity=0.4,
    )
    segment = slice_trajectory(trajectory, 0.5, 1.5)
    assert segment.times == (0.0, 0.5, 1.0)
    assert [frame[0] for frame in segment.positions] == pytest.approx(
        (0.1, 0.2, 0.4)
    )
