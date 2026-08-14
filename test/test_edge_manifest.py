from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

from graspv2.edge_manifest import EdgeManifestError, load_edge_manifest, sha256_file
from graspv2.remote_bundle import build_remote_bundle


def _write_bundle(tmp_path: Path) -> Path:
    animation = tmp_path / "grasp_animation.csv"
    animation.write_text("timeMS,command_pos::joint\n0,0\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "kind": "graspv2-x2-edge-execution",
        "robot": {"profile": "ultra", "arm_side": "right"},
        "target_class": "cup",
        "animation": {
            "file": animation.name,
            "sha256": sha256_file(animation),
            "duration_s": 5.0,
            "maximum_arm_velocity_rad_s": 0.3,
            "initial_gripper": {"position": 1.0, "duration_s": 3.0},
            "gripper_events": [
                {"time_s": 1.0, "position": 0.2, "label": "close"},
                {"time_s": 4.0, "position": 1.0, "label": "release"},
            ],
            "return_path_enabled": True,
        },
        "execution_limitations": {
            "atomic_mc_animation": True,
            "mid_motion_visual_gates": False,
        },
    }
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def test_edge_manifest_verifies_checksum_and_gripper_contract(
    tmp_path: Path,
) -> None:
    path = _write_bundle(tmp_path)

    manifest = load_edge_manifest(path)

    assert manifest.robot_profile == "ultra"
    assert manifest.arm_side == "right"
    assert manifest.target_class == "cup"
    assert [event.label for event in manifest.gripper_events] == ["close", "release"]


def test_edge_manifest_rejects_animation_modified_after_planning(tmp_path: Path) -> None:
    path = _write_bundle(tmp_path)
    (tmp_path / "grasp_animation.csv").write_text("tampered\n", encoding="utf-8")

    with pytest.raises(EdgeManifestError, match="SHA-256 mismatch"):
        load_edge_manifest(path)


def test_edge_manifest_rejects_non_monotonic_events(tmp_path: Path) -> None:
    path = _write_bundle(tmp_path)
    document = json.loads(path.read_text(encoding="utf-8"))
    document["animation"]["gripper_events"][1]["time_s"] = 1.0
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(EdgeManifestError, match="strictly increasing"):
        load_edge_manifest(path)


def _trajectory_document(
    positions: list[float],
    times: list[float],
    planning: dict[str, object],
) -> dict[str, object]:
    joint = "right_shoulder_pitch_joint"
    return {
        "format": "x2_active_joint_trajectory",
        "units": {"time": "s", "joint_angle": "rad"},
        "robot_profile": "ultra",
        "arm_side": "right",
        "active_joint_names": [joint],
        "frame_count": len(times),
        "duration_s": times[-1],
        "frames": [
            {
                "frame": index,
                "time_s": time_s,
                "active_joints_rad": {joint: position},
            }
            for index, (time_s, position) in enumerate(zip(times, positions))
        ],
        "planning": {
            "verified_collision_free": True,
            "ik_backend": "x2_ik_sdk.X2ArmIKSolver",
            **planning,
        },
    }


def test_local_builder_produces_robot_verifiable_minimal_bundle(
    tmp_path: Path,
) -> None:
    common_sequence = {
        "visual_radius_m": 0.04,
        "preopen_position": 1.0,
        "grip_position": 0.6,
        "vertical_descent_start_time_s": 1.0,
        "open_duration_s": 3.0,
        "close_duration_s": 0.8,
        "grasp_settle_duration_s": 0.3,
        "lifted_hold_duration_s": 0.5,
        "controlled_lower_duration_s": 2.0,
        "open_hand_retreat_duration_s": 0.5,
        "release_duration_s": 0.6,
        "place_settle_duration_s": 0.7,
        "reclose_duration_s": 0.6,
        "gripper_fully_open_before_arm_motion": True,
    }
    approach = tmp_path / "approach.json"
    approach.write_text(
        json.dumps(
            _trajectory_document(
                [0.4, 0.35, 0.3],
                [0.0, 1.0, 2.0],
                {
                    "trajectory_role": "approach",
                    "object_center_world_m": [0.4, -0.2, 0.795],
                    "gripper_center_world_m": [0.4, -0.2, 0.8],
                    "target_world_m": [0.4, -0.2, 0.8],
                    "vertical_descent_start_time_s": 1.0,
                    "gripper_fully_open_before_arm_motion": True,
                    "simulated_grasp_sequence": common_sequence,
                },
            )
        ),
        encoding="utf-8",
    )
    lift = tmp_path / "lift.json"
    lift.write_text(
        json.dumps(
            _trajectory_document(
                [0.3, 0.2],
                [0.0, 2.0],
                {
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
            )
        ),
        encoding="utf-8",
    )
    return_path = tmp_path / "return.json"
    return_path.write_text(
        json.dumps(
            _trajectory_document(
                [0.2, 0.3, 0.35, 0.4],
                [0.0, 2.0, 2.5, 4.0],
                {
                    "trajectory_role": "return_to_default",
                    "return_mode": "controlled_lower_then_reverse_approach",
                    "controlled_lower_duration_s": 2.0,
                    "open_hand_retreat_duration_s": 0.5,
                },
            )
        ),
        encoding="utf-8",
    )
    vision = tmp_path / "result.json"
    vision.write_text(
        json.dumps(
            {
                "schema_version": 2,
                "object_center_mujoco_m": [0.4, -0.2, 0.795],
                "selected_detection": {
                    "class_name": "cup",
                    "confidence": 0.9,
                },
            }
        ),
        encoding="utf-8",
    )

    manifest_path = build_remote_bundle(
        argparse.Namespace(
            approach_trajectory=approach,
            lift_trajectory=lift,
            return_trajectory=return_path,
            initial_vision=vision,
            target_class="cup",
            output_dir=tmp_path / "edge",
        )
    )

    manifest = load_edge_manifest(manifest_path)
    assert manifest.animation_path.name == "grasp_animation.csv"
    assert len(manifest.gripper_events) == 3
    assert manifest.initial_gripper_duration_s == pytest.approx(3.0)
