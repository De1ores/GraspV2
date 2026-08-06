from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from graspv2 import cli
from graspv2.mc_animation import (
    DEFAULT_ARM_POSITION,
    MC_JOINT_ORDER,
    build_mc_animation,
)
from graspv2.official_ik import OfficialIK
from graspv2.planner import plan_lift_trajectory, plan_trajectory
from graspv2.robot_profiles import OPTIONAL_WRISTS, PROFILES
from graspv2.simulation import (
    RobotSimulation,
    load_table_obstacle,
    resolve_robot_visual_urdf,
    validate_fk_alignment,
)
from graspv2.trajectory import load_trajectory


FIXTURE = Path(__file__).parent / "fixtures" / "reachable_table.json"


@pytest.mark.parametrize("profile_name,arm_dof", [("youth", 5), ("ultra", 7)])
def test_official_sdk_and_mujoco_have_identical_kinematics(
    profile_name: str, arm_dof: int
) -> None:
    profile = PROFILES[profile_name]
    ik = OfficialIK(profile)
    assert profile.arm_dof == arm_dof
    assert len(ik.ready_arm_pos()) == arm_dof * 2
    alignment = validate_fk_alignment(profile, ik, random_samples=5)
    assert alignment.maximum_position_error_m < 1e-9
    assert alignment.maximum_orientation_error_deg < 1e-4
    simulation = RobotSimulation(profile, ik)
    assert simulation.collision_report().valid


def test_configured_tcp_pose_is_shared_by_ik_and_mujoco(tmp_path: Path) -> None:
    profile = PROFILES["youth"]
    calibration = {
        "schema_version": 1,
        "left": {
            "parent_frame": profile.left_ee_frame,
            "translation_m": [0.025, -0.01, 0.04],
            "rpy_rad": [0.05, -0.10, 0.20],
        },
        "right": {
            "parent_frame": profile.right_ee_frame,
            "translation_m": [0.03, 0.012, 0.045],
            "rpy_rad": [-0.04, 0.08, -0.18],
        },
    }
    calibration_path = tmp_path / "tool_pose_offset.json"
    calibration_path.write_text(json.dumps(calibration), encoding="utf-8")
    base_ik = OfficialIK(profile)
    calibrated_ik = OfficialIK(profile, calibration_path)
    arm_pos = profile.mc_start_arm_pos()
    assert np.linalg.norm(
        np.asarray(calibrated_ik.fk_world("right", arm_pos))
        - np.asarray(base_ik.fk_world("right", arm_pos))
    ) > 0.04
    simulation = RobotSimulation(profile, calibrated_ik)
    simulation.set_arm_pos(arm_pos)
    assert simulation.site_world_xyz("right") == pytest.approx(
        calibrated_ik.fk_world("right", arm_pos), abs=1e-9
    )
    target = np.asarray(calibrated_ik.fk_world("right", arm_pos)) + [0.01, 0.0, 0.0]
    solved = calibrated_ik.solve_world_position("right", target, arm_pos)
    assert solved.success
    assert solved.final_world_xyz == pytest.approx(target, abs=1e-4)
    alignment = validate_fk_alignment(profile, calibrated_ik, random_samples=3)
    assert alignment.maximum_position_error_m < 1e-9
    assert alignment.maximum_orientation_error_deg < 1e-4


def test_future_upper_profile_remains_safely_disabled() -> None:
    with pytest.raises(RuntimeError, match="competition URDF"):
        OfficialIK(PROFILES["future_upper"])


def test_installed_physical_urdf_loads_full_body_visual_meshes() -> None:
    visual_urdf = resolve_robot_visual_urdf()
    if visual_urdf is None:
        pytest.skip("physical X2 visual URDF is not installed")
    profile = PROFILES["ultra"]
    simulation = RobotSimulation(
        profile,
        OfficialIK(profile),
        visual_urdf_path=visual_urdf,
    )
    assert simulation.visual_urdf_path == visual_urdf
    assert simulation.omnipicker_description_path is not None
    assert simulation.model.nmesh >= 38
    assert "actual_visual_pelvis_0" in simulation.xml
    assert "left_ankle_roll_link" in simulation.xml
    assert "right_ankle_roll_link" in simulation.xml
    assert "actual_visual_L_omnipicker_base_link_0" in simulation.xml
    assert "actual_visual_R_omnipicker_base_link_0" in simulation.xml
    assert "actual_visual_L_hand_narrow_loop_Link_0" in simulation.xml
    assert "actual_visual_R_hand_wide_loop_Link_0" in simulation.xml
    assert "actual_visual_left_wrist_roll_link_0" not in simulation.xml
    assert "actual_visual_right_wrist_roll_link_0" not in simulation.xml
    assert "fist" not in simulation.xml.lower()
    assert "proxy_left_omnipicker" not in simulation.xml
    assert "proxy_right_omnipicker" not in simulation.xml


def test_scene_contains_floor_lighting_and_complete_visual_table() -> None:
    profile = PROFILES["youth"]
    _, obstacle = load_table_obstacle(FIXTURE)
    simulation = RobotSimulation(profile, OfficialIK(profile), obstacle)
    assert simulation.model.nlight == 2
    assert simulation.table_geom_id >= 0
    assert 'name="scene_floor"' in simulation.xml
    assert 'type="skybox"' in simulation.xml
    assert 'name="scene_key_light"' in simulation.xml
    assert 'name="scene_fill_light"' in simulation.xml
    assert 'name="planning_table_leg_3"' in simulation.xml


def test_recognized_target_object_is_displayed_on_the_table(tmp_path: Path) -> None:
    document = json.loads(FIXTURE.read_text(encoding="utf-8"))
    document["selected_detection"] = {
        "class_name": "bottle",
        "confidence": 0.93,
    }
    result_path = tmp_path / "bottle_result.json"
    result_path.write_text(json.dumps(document), encoding="utf-8")
    _, obstacle = load_table_obstacle(result_path)
    assert obstacle.target_object is not None
    assert obstacle.target_object.class_name == "bottle"
    assert obstacle.target_object.geom_type == "cylinder"
    assert obstacle.target_object.center_m[2] == pytest.approx(0.9)

    profile = PROFILES["youth"]
    simulation = RobotSimulation(profile, OfficialIK(profile), obstacle)
    assert simulation.target_object_geom_id >= 0
    assert 'name="recognized_target_object"' in simulation.xml
    target_geom = simulation.target_object_geom_id
    assert simulation.model.geom_contype[target_geom] == 0
    assert simulation.model.geom_conaffinity[target_geom] == 0


@pytest.mark.parametrize("profile_name", ["youth", "ultra"])
def test_table_plan_is_verified_and_loadable(
    profile_name: str, tmp_path: Path
) -> None:
    profile = PROFILES[profile_name]
    result = plan_trajectory(profile, vision_result=FIXTURE)
    assert result.obstacle is not None
    assert result.report["ik_backend"] == "x2_ik_sdk.X2ArmIKSolver"
    assert result.report["verified_collision_free"] is True
    assert result.report["minimum_observed_table_distance_m"] >= 0.025
    assert result.report["final_position_error_m"] < 1e-3
    trajectory_path = tmp_path / "trajectory.json"
    report_path = tmp_path / "report.json"
    result.write(trajectory_path, report_path)
    loaded = load_trajectory(
        trajectory_path,
        maximum_allowed_velocity=profile.maximum_velocity_rad_s + 1e-6,
    )
    assert loaded.joint_names == profile.right_arm_joints
    assert loaded.frame_count == len(result.times)
    document = json.loads(trajectory_path.read_text(encoding="utf-8"))
    assert document["robot_profile"] == profile_name
    assert document["planning"]["table_obstacle"]["center_m"] == pytest.approx(
        result.obstacle.center_m
    )


def test_cli_uses_latest_vision_result_unless_explicitly_disabled(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    latest = tmp_path / "result.json"
    latest.write_text("{}\n", encoding="utf-8")
    monkeypatch.setattr(cli, "DEFAULT_VISION_RESULT", latest)
    default_args = cli._parser().parse_args([])
    assert cli._resolve_vision_result(default_args) == latest
    assert default_args.headless is False
    assert default_args.side == "auto"
    no_vision_args = cli._parser().parse_args(["--no-vision"])
    assert cli._resolve_vision_result(no_vision_args) is None
    headless_args = cli._parser().parse_args(["--headless"])
    assert headless_args.headless is True


def test_default_auto_side_retries_the_other_arm(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    args = cli._parser().parse_args(["--no-vision"])
    selected = object()
    attempts: list[str] = []

    def fake_plan(profile, *, side, **kwargs):
        attempts.append(side)
        if side == "right":
            raise RuntimeError("right target unreachable")
        return selected

    monkeypatch.setattr(cli, "plan_trajectory", fake_plan)
    result = cli._plan_with_side_fallback(args, PROFILES["youth"], None)
    assert result is selected
    assert attempts == ["right", "left"]


def test_auto_side_prefers_the_target_half_space() -> None:
    parser = cli._parser()
    left_target = parser.parse_args(["--target", "0.5", "0.2", "0.9"])
    right_target = parser.parse_args(["--target", "0.5", "-0.2", "0.9"])
    assert cli._automatic_side_order(left_target, None) == ("left", "right")
    assert cli._automatic_side_order(right_target, None) == ("right", "left")
    forced = parser.parse_args(["--side", "right"])
    assert cli._automatic_side_order(forced, None) == ("right",)


def test_sim_opens_viewer_by_default_and_headless_skips_it(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    planned = object()
    opened: list[object] = []
    monkeypatch.setattr(cli, "_plan", lambda args, profile: planned)
    monkeypatch.setattr(cli, "replay_viewer", opened.append)
    assert cli.main([]) == 0
    assert opened == [planned]
    opened.clear()
    assert cli.main(["--headless"]) == 0
    assert opened == []


def test_failed_plan_still_opens_default_static_scene(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previewed: list[tuple[object, Path | None]] = []

    def fail_plan(args, profile):
        raise RuntimeError("unreachable test target")

    def record_preview(profile, *, vision_result, robot_urdf=None):
        previewed.append((profile, vision_result))

    monkeypatch.setattr(cli, "_plan", fail_plan)
    monkeypatch.setattr(cli, "_resolve_vision_result", lambda args: FIXTURE)
    monkeypatch.setattr(cli, "preview_scene", record_preview)
    assert cli.main([]) == 1
    assert previewed == [(PROFILES["youth"], FIXTURE)]
    previewed.clear()
    assert cli.main(["--headless"]) == 1
    assert previewed == []


def test_youth_animation_keeps_absent_wrist_columns_fixed(tmp_path: Path) -> None:
    profile = PROFILES["youth"]
    result = plan_trajectory(profile)
    trajectory_path = tmp_path / "youth.json"
    result.write(trajectory_path, tmp_path / "report.json")
    trajectory = load_trajectory(trajectory_path)
    animation = build_mc_animation(
        trajectory,
        maximum_output_velocity=profile.maximum_velocity_rad_s,
    )
    assert trajectory.positions[0] == pytest.approx(
        profile.mc_start_arm_pos()[profile.arm_dof:]
    )
    lead_in_rows = [
        row for row in animation.rows if row[0] <= animation.bridge_duration_s * 1000.0
    ]
    assert all(
        row[4:18] == pytest.approx(DEFAULT_ARM_POSITION) for row in lead_in_rows
    )
    defaults = dict(zip(MC_JOINT_ORDER[3:17], DEFAULT_ARM_POSITION))
    for joint_name in OPTIONAL_WRISTS:
        column = 1 + MC_JOINT_ORDER.index(joint_name)
        assert all(row[column] == pytest.approx(defaults[joint_name]) for row in animation.rows)


def test_clearance_gate_rejects_same_path_at_stricter_threshold() -> None:
    profile = PROFILES["youth"]
    baseline = plan_trajectory(
        profile,
        vision_result=FIXTURE,
        table_clearance_m=0.025,
    )
    assert baseline.report["minimum_observed_table_distance_m"] < 0.08
    simulation = RobotSimulation(
        profile,
        OfficialIK(profile),
        baseline.obstacle,
        visual_urdf_path=baseline.visual_urdf_path,
    )
    start_arm_pos = profile.mc_start_arm_pos()
    reports = []
    for positions in baseline.positions:
        simulation.set_side_joints(
            baseline.side,
            positions,
            start_arm_pos,
        )
        reports.append(simulation.collision_report(0.08))
    assert any(not report.valid for report in reports)

    replanned = plan_trajectory(
        profile,
        vision_result=FIXTURE,
        table_clearance_m=0.08,
    )
    assert replanned.report["verified_collision_free"] is True
    assert replanned.report["minimum_observed_table_distance_m"] >= 0.08


def test_lift_plan_starts_at_grasp_and_runs_for_two_seconds() -> None:
    approach = plan_trajectory(
        PROFILES["youth"],
        vision_result=FIXTURE,
        table_clearance_m=0.025,
    )
    lift = plan_lift_trajectory(
        approach,
        lift_height_m=0.08,
        lift_duration_s=2.0,
    )
    assert lift.positions[0] == pytest.approx(approach.positions[-1])
    assert lift.duration_s == pytest.approx(2.0)
    assert lift.report["trajectory_role"] == "lift"
    assert lift.report["verified_collision_free"] is True
    assert lift.report["lift_height_m"] == pytest.approx(0.08)
    assert lift.report["minimum_observed_table_distance_m"] >= 0.025
