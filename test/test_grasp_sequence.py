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
)


def _observation(path: Path, point: tuple[float, float, float]) -> VisualObservation:
    return VisualObservation(
        source=path,
        class_name="bottle",
        confidence=0.9,
        point_world_m=point,
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
        grasp_target_world_m=(0.40, -0.20, 0.80),
        lifted_target_world_m=(0.40, -0.20, 0.90),
        lift_direction_world=(0.0, 0.0, 1.0),
        lift_height_m=0.10,
        lift_duration_s=2.0,
    )
    closed = _observation(tmp_path / "closed.json", (0.405, -0.20, 0.802))
    closed_result = verify_closed_observation(
        closed,
        metadata.grasp_target_world_m,
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
                "grasp_point_mujoco_m": [0.4, -0.2, 0.8],
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
                    "target_world_m": [0.4, -0.2, 0.8],
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
    metadata = load_grasp_plan_metadata(approach_path, lift_path)
    assert metadata.lift_duration_s == pytest.approx(2.0)

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
