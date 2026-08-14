from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from graspv2 import cli, simulation as simulation_module
from graspv2.mc_animation import (
    DEFAULT_ARM_POSITION,
    MC_JOINT_ORDER,
    build_mc_animation,
)
from graspv2.official_ik import OfficialIK, WorldIKResult
from graspv2.planner import (
    PlannedTrajectory,
    _interpolate_replay_position,
    gripper_positions_for_visual_radius,
    plan_lift_trajectory,
    plan_simulated_grasp_sequence,
    plan_trajectory,
    solve_collision_free_ik,
)
from graspv2.robot_profiles import PROFILES
from graspv2.simulation import (
    RIGHT_CLAW_JOINT_NAME,
    RIGHT_WIDE_JOINT_NAME,
    RobotSimulation,
    load_table_obstacle,
    resolve_robot_visual_urdf,
    validate_fk_alignment,
)
from graspv2.trajectory import TrajectoryValidationError, load_trajectory


FIXTURE = Path(__file__).parent / "fixtures" / "reachable_table.json"
DEMO_SCENE = Path(__file__).parents[1] / "config" / "mujoco_demo_scene.json"


def _failed_world_ik(position_error_m: float, joint: float) -> WorldIKResult:
    target = (0.0, 0.0, 0.0)
    return WorldIKResult(
        sdk_result=SimpleNamespace(
            success=False,
            arm_pos=[0.0, joint],
            active_arm=[joint],
            error_norm=position_error_m,
            message="max iterations reached",
        ),
        target_world_xyz=target,
        final_world_xyz=(position_error_m, 0.0, 0.0),
    )


def test_nearest_ik_fallback_selects_closest_collision_free_seed() -> None:
    class FakeIK:
        profile = SimpleNamespace(arm_dof=1)

        def __init__(self):
            self.results = iter(
                (
                    _failed_world_ik(0.040, 0.1),
                    _failed_world_ik(0.010, 0.2),
                    _failed_world_ik(0.020, 0.3),
                )
            )

        def joint_limits_for_side(self, _side):
            return np.asarray([-1.0]), np.asarray([1.0])

        def solve_world_position(self, _side, _target, _seed):
            return next(self.results)

    checker = SimpleNamespace(
        state_valid=lambda _active: True,
        edge_valid=lambda _start, _active: True,
    )
    selected = solve_collision_free_ik(
        FakeIK(),
        checker,
        "right",
        (0.0, 0.0, 0.0),
        [0.0, 0.0],
        np.random.default_rng(1),
        attempts=3,
        required_edge_start=np.asarray([0.0]),
    )

    assert selected.success
    assert selected.accepted_nearest
    assert selected.position_error_m == pytest.approx(0.010)
    assert selected.active_arm == [0.2]


def test_nearest_ik_fallback_keeps_collision_and_five_cm_gates() -> None:
    class FakeIK:
        profile = SimpleNamespace(arm_dof=1)

        def __init__(self, results):
            self.results = iter(results)

        def joint_limits_for_side(self, _side):
            return np.asarray([-1.0]), np.asarray([1.0])

        def solve_world_position(self, _side, _target, _seed):
            return next(self.results)

    checker = SimpleNamespace(
        state_valid=lambda active: float(active[0]) != 0.1,
        edge_valid=lambda _start, _active: True,
    )
    selected = solve_collision_free_ik(
        FakeIK(
            (
                _failed_world_ik(0.005, 0.1),
                _failed_world_ik(0.020, 0.2),
            )
        ),
        checker,
        "right",
        (0.0, 0.0, 0.0),
        [0.0, 0.0],
        np.random.default_rng(2),
        attempts=2,
        required_edge_start=np.asarray([0.0]),
    )
    assert selected.active_arm == [0.2]
    assert selected.position_error_m == pytest.approx(0.020)

    with pytest.raises(RuntimeError, match="best position error=0.0501 m"):
        solve_collision_free_ik(
            FakeIK((_failed_world_ik(0.0501, 0.2),)),
            checker,
            "right",
            (0.0, 0.0, 0.0),
            [0.0, 0.0],
            np.random.default_rng(3),
            attempts=1,
            required_edge_start=np.asarray([0.0]),
        )


def test_official_sdk_and_mujoco_have_identical_ultra_kinematics() -> None:
    profile = PROFILES["ultra"]
    ik = OfficialIK(profile)
    assert profile.arm_dof == 7
    assert len(ik.ready_arm_pos()) == 14
    alignment = validate_fk_alignment(profile, ik, random_samples=5)
    assert alignment.maximum_position_error_m < 1e-9
    assert alignment.maximum_orientation_error_deg < 1e-4
    simulation = RobotSimulation(profile, ik)
    assert simulation.collision_report().valid


def test_configured_tcp_pose_is_shared_by_ik_and_mujoco(tmp_path: Path) -> None:
    profile = PROFILES["ultra"]
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
    observed_translation_delta = np.linalg.norm(
        np.asarray(calibrated_ik.fk_world("right", arm_pos))
        - np.asarray(base_ik.fk_world("right", arm_pos))
    )
    expected_translation_delta = np.linalg.norm(
        np.asarray(calibration["right"]["translation_m"])
        - np.asarray(base_ik.tool_pose.right.translation_m)
    )
    assert observed_translation_delta == pytest.approx(
        expected_translation_delta, abs=1e-9
    )
    assert expected_translation_delta > 0.0
    simulation = RobotSimulation(profile, calibrated_ik)
    simulation.set_arm_pos(arm_pos)
    assert simulation.site_world_xyz("right") == pytest.approx(
        calibrated_ik.fk_world("right", arm_pos), abs=1e-9
    )
    target = np.asarray(calibrated_ik.fk_world("right", arm_pos)) + [0.01, 0.0, 0.0]
    solved = calibrated_ik.solve_world_position("right", target, arm_pos)
    assert solved.success
    assert solved.final_world_xyz == pytest.approx(
        target, abs=calibrated_ik.solver.config.eps
    )
    alignment = validate_fk_alignment(profile, calibrated_ik, random_samples=3)
    assert alignment.maximum_position_error_m < 1e-9
    assert alignment.maximum_orientation_error_deg < 1e-4


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
    assert "actual_visual_L_omnipicker_base_link_0" not in simulation.xml
    assert "actual_visual_R_omnipicker_base_link_0" in simulation.xml
    assert "actual_visual_L_hand_narrow_loop_Link_0" not in simulation.xml
    assert "actual_visual_R_hand_wide_loop_Link_0" in simulation.xml
    assert "actual_visual_right_wrist_roll_link_0" not in simulation.xml
    assert "proxy_left_omnipicker" not in simulation.xml
    assert "proxy_right_omnipicker" not in simulation.xml


def test_incomplete_auto_discovered_visual_urdf_uses_proxy_fallback(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    broken = tmp_path / "x2_whole_body.urdf"
    broken.write_text(
        """<robot name="x2">
  <link name="pelvis">
    <visual><geometry><mesh filename="./meshes/pelvis.STL"/></geometry></visual>
  </link>
</robot>
""",
        encoding="utf-8",
    )
    monkeypatch.delenv(simulation_module.ROBOT_URDF_ENV, raising=False)
    monkeypatch.setattr(
        simulation_module,
        "_robot_urdf_candidates",
        lambda: (broken,),
    )

    assert simulation_module.resolve_robot_visual_urdf() is None
    with pytest.raises(ValueError, match="robot visual mesh does not exist"):
        simulation_module.resolve_robot_visual_urdf(broken)


def test_scene_contains_floor_lighting_and_complete_visual_table() -> None:
    profile = PROFILES["ultra"]
    _, obstacle = load_table_obstacle(FIXTURE)
    simulation = RobotSimulation(profile, OfficialIK(profile), obstacle)
    assert simulation.model.nlight == 2
    assert simulation.table_geom_id >= 0
    assert 'name="scene_floor"' in simulation.xml
    assert 'type="skybox"' in simulation.xml
    assert 'name="scene_key_light"' in simulation.xml
    assert 'name="scene_fill_light"' in simulation.xml
    assert 'name="planning_table_leg_3"' in simulation.xml


def test_demo_scene_uses_full_ultra_proxy_and_right_omnipicker() -> None:
    target, obstacle = load_table_obstacle(DEMO_SCENE)
    simulation = RobotSimulation(
        PROFILES["ultra"],
        OfficialIK(PROFILES["ultra"]),
        obstacle,
    )
    assert target == pytest.approx((0.3666481438, -0.3553605768, 0.91))
    assert simulation.table_geom_id >= 0
    assert simulation.target_object_geom_id >= 0
    assert 'name="proxy_visual_pelvis"' in simulation.xml
    assert 'name="proxy_visual_left_knee_link"' in simulation.xml
    assert 'name="proxy_visual_right_ankle_roll_link"' in simulation.xml
    assert 'name="proxy_visual_head"' in simulation.xml
    assert "actual_visual_R_omnipicker_base_link_0" in simulation.xml
    assert "actual_visual_L_omnipicker_base_link_0" not in simulation.xml


def test_demo_scene_completes_approach_lift_and_return_planning() -> None:
    approach = plan_trajectory(PROFILES["ultra"], vision_result=DEMO_SCENE)
    sequence = plan_simulated_grasp_sequence(approach)

    assert approach.report["verified_collision_free"] is True
    assert sequence.lift.report["verified_collision_free"] is True
    assert sequence.return_to_default.report["verified_collision_free"] is True
    assert sequence.return_to_default.positions[-1] == pytest.approx(
        PROFILES["ultra"].mc_start_arm_pos()[PROFILES["ultra"].arm_dof :]
    )
    assert approach.report["pregrasp_clearance_above_object_m"] == pytest.approx(
        0.03
    )
    assert (
        approach.target_world_xyz[2]
        - approach.obstacle.target_object.object_center_m[2]
    ) == pytest.approx(0.01)
    assert sequence.preopen_position == pytest.approx(1.0)
    assert sequence.open_duration_s == pytest.approx(3.0)
    assert approach.report["gripper_fully_open_before_arm_motion"] is True
    assert sequence.report["gripper_fully_open_before_arm_motion"] is True
    assert sequence.report["phases"][0] == (
        "fully_open_gripper_before_arm_motion"
    )
    assert sequence.report["lifted_hold_duration_s"] == pytest.approx(2.5)
    assert sequence.report["controlled_lower_duration_s"] == pytest.approx(
        sequence.lift.duration_s
    )
    assert sequence.return_to_default.report["return_mode"] == (
        "controlled_lower_then_reverse_approach"
    )


def test_right_omnipicker_opening_and_target_motion_are_simulated() -> None:
    _, obstacle = load_table_obstacle(DEMO_SCENE)
    simulation = RobotSimulation(
        PROFILES["ultra"],
        OfficialIK(PROFILES["ultra"]),
        obstacle,
    )
    simulation.set_gripper_position(1.0)
    assert simulation.data.qpos[
        simulation.gripper_qpos_indices[RIGHT_CLAW_JOINT_NAME]
    ] == pytest.approx(-1.0)
    assert simulation.data.qpos[
        simulation.gripper_qpos_indices[RIGHT_WIDE_JOINT_NAME]
    ] == pytest.approx(1.0)
    simulation.set_gripper_position(0.0)
    assert simulation.data.qpos[
        simulation.gripper_qpos_indices[RIGHT_CLAW_JOINT_NAME]
    ] == pytest.approx(0.0)
    moved = (0.40, -0.30, 1.00)
    simulation.set_target_object_pose(moved)
    assert simulation.target_object_pose()[0] == pytest.approx(moved)


def test_visual_radius_controls_preopen_and_grasp_opening() -> None:
    preopen, grip = gripper_positions_for_visual_radius(0.045)
    assert preopen == pytest.approx(1.0)
    assert grip == pytest.approx(0.7166666667)
    assert 0.0 <= grip < preopen <= 1.0


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
    assert obstacle.target_object.object_center_m[2] == pytest.approx(0.9)
    assert obstacle.target_object.gripper_center_m[2] == pytest.approx(0.91)

    profile = PROFILES["ultra"]
    simulation = RobotSimulation(profile, OfficialIK(profile), obstacle)
    assert simulation.target_object_geom_id >= 0
    assert 'name="recognized_target_object"' in simulation.xml
    target_geom = simulation.target_object_geom_id
    assert simulation.model.geom_contype[target_geom] == 0
    assert simulation.model.geom_conaffinity[target_geom] == 0


def test_schema_v1_surface_point_is_read_only_as_legacy_input(tmp_path: Path) -> None:
    document = json.loads(FIXTURE.read_text(encoding="utf-8"))
    surface = document.pop("surface_point_mujoco_m")
    document.pop("object_center_mujoco_m")
    document.pop("gripper_center_mujoco_m")
    document["schema_version"] = 1
    document["grasp_point_mujoco_m"] = surface
    legacy_path = tmp_path / "legacy_result.json"
    legacy_path.write_text(json.dumps(document), encoding="utf-8")

    gripper_center, obstacle = load_table_obstacle(legacy_path)
    assert obstacle.target_object is not None
    assert obstacle.target_object.object_center_m[2] == pytest.approx(0.9)
    assert obstacle.target_object.gripper_center_m[2] == pytest.approx(0.91)
    assert gripper_center == pytest.approx(obstacle.target_object.gripper_center_m)


def test_table_plan_is_verified_and_loadable(tmp_path: Path) -> None:
    profile = PROFILES["ultra"]
    result = plan_trajectory(profile, vision_result=FIXTURE)
    assert result.obstacle is not None
    assert result.report["ik_backend"] == "x2_ik_sdk.X2ArmIKSolver"
    assert result.report["verified_collision_free"] is True
    assert result.report["minimum_observed_table_distance_m"] >= 0.025
    assert result.report["final_position_error_m"] < 1e-3
    assert result.report["planning_strategy"] == (
        "robot_side_safe_staging_then_cartesian_grasp"
    )
    assert "rrt" not in result.report
    default_tcp = OfficialIK(profile).fk_world(
        "right", profile.mc_start_arm_pos()
    )
    safe_staging = result.report["safe_staging_world_m"]
    assert safe_staging[0] == pytest.approx(default_tcp[0])
    assert safe_staging[1] == pytest.approx(default_tcp[1] - 0.06)
    assert safe_staging[2] == pytest.approx(1.03)
    assert result.report["safe_staging_path_maximum_deviation_m"] < 0.01
    assert result.report["safe_staging_path_minimum_forward_step_m"] >= -1e-4
    assert result.report["safe_staging_final_error_m"] < 1e-3
    assert result.report["safe_transfer_maximum_height_error_m"] < 0.01
    descent_start = result.report["vertical_descent_start_time_s"]
    assert 0.0 < descent_start < result.duration_s
    assert result.report["object_top_world_m"][2] == pytest.approx(1.0)
    assert result.pregrasp_world_xyz[2] == pytest.approx(1.03)
    assert (
        result.pregrasp_world_xyz[2]
        - result.report["object_top_world_m"][2]
    ) == pytest.approx(0.03)
    assert (
        result.target_world_xyz[2]
        - result.obstacle.target_object.object_center_m[2]
    ) == pytest.approx(0.01)
    assert result.report["grasp_mode"] == "position_only_side_grasp"
    assert result.report["orientation_ik_enabled"] is False
    assert result.report["gripper_side_approach_local_axis"] is None
    assert result.report["grasp_axis_world"] is None
    assert result.target_world_xyz[2] == pytest.approx(
        result.obstacle.target_object.gripper_center_m[2]
    )
    assert result.report["maximum_side_approach_table_tilt_deg"] is None
    assert result.report["maximum_opening_direction_table_tilt_deg"] is None
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
    assert document["robot_profile"] == "ultra"
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
    assert default_args.side == "right"
    no_vision_args = cli._parser().parse_args(["--no-vision"])
    assert cli._resolve_vision_result(no_vision_args) is None
    demo_args = cli._parser().parse_args(["--demo-scene"])
    assert cli._resolve_vision_result(demo_args) == DEMO_SCENE
    headless_args = cli._parser().parse_args(["--headless"])
    assert headless_args.headless is True


def test_right_only_planning_never_falls_back_to_left(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    args = cli._parser().parse_args(["--no-vision"])
    attempts: list[str] = []

    def fake_plan(profile, *, side, **kwargs):
        attempts.append(side)
        raise RuntimeError("right target unreachable")

    monkeypatch.setattr(cli, "plan_trajectory", fake_plan)
    with pytest.raises(RuntimeError, match="right target unreachable"):
        cli._plan_with_installed_gripper(args, PROFILES["ultra"], None)
    assert attempts == ["right"]


def test_grasp_side_is_right_even_for_a_left_half_space_target() -> None:
    parser = cli._parser()
    left_target = parser.parse_args(["--target", "0.5", "0.2", "0.9"])
    right_target = parser.parse_args(["--target", "0.5", "-0.2", "0.9"])
    assert cli._grasp_side(left_target) == "right"
    assert cli._grasp_side(right_target) == "right"
    forced = parser.parse_args(["--side", "right"])
    assert cli._grasp_side(forced) == "right"
    with pytest.raises(SystemExit):
        parser.parse_args(["--side", "left"])


def test_direct_left_grasp_plan_is_rejected() -> None:
    with pytest.raises(ValueError, match="no left OmniPicker"):
        plan_trajectory(PROFILES["ultra"], side="left")


def test_existing_left_arm_trajectory_cannot_enter_grasp_playback(
    tmp_path: Path,
) -> None:
    path = tmp_path / "left_trajectory.json"
    path.write_text(
        json.dumps(
            {
                "robot_profile": "ultra",
                "arm_side": "left",
                "planning": {
                    "verified_collision_free": True,
                    "ik_backend": "x2_ik_sdk.X2ArmIKSolver",
                },
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(TrajectoryValidationError, match="no left OmniPicker"):
        cli._validate_trajectory_profile(path, PROFILES["ultra"])


def test_mujoco_scene_contains_only_the_installed_right_omnipicker() -> None:
    profile = PROFILES["ultra"]
    simulation = RobotSimulation(profile, OfficialIK(profile))
    assert "actual_visual_R_omnipicker_base_link_0" in simulation.xml
    assert "actual_visual_L_omnipicker_base_link_0" not in simulation.xml


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


def test_viewer_interpolates_by_wall_clock_and_skips_render_frames() -> None:
    trajectory = PlannedTrajectory(
        profile=PROFILES["ultra"],
        side="right",
        obstacle=None,
        visual_urdf_path=None,
        target_world_xyz=(0.0, 0.0, 0.0),
        pregrasp_world_xyz=(0.0, 0.0, 0.0),
        joint_names=("joint",),
        times=(0.0, 0.5, 1.0),
        positions=((0.0,), (1.0,), (0.0,)),
        report={},
    )
    assert _interpolate_replay_position(trajectory, -1.0) == (0.0,)
    assert _interpolate_replay_position(trajectory, 0.25) == pytest.approx((0.5,))
    assert _interpolate_replay_position(trajectory, 0.75) == pytest.approx((0.5,))
    assert _interpolate_replay_position(trajectory, 2.0) == (0.0,)


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
    assert previewed == [(PROFILES["ultra"], FIXTURE)]
    previewed.clear()
    assert cli.main(["--headless"]) == 1
    assert previewed == []


def test_ultra_animation_uses_the_14_joint_arm_layout(tmp_path: Path) -> None:
    profile = PROFILES["ultra"]
    result = plan_trajectory(profile)
    trajectory_path = tmp_path / "ultra.json"
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
    assert len(profile.arm_pos_order) == 14
    assert set(trajectory.joint_names) == set(profile.right_arm_joints)
    assert all(len(row) == len(MC_JOINT_ORDER) + 1 for row in animation.rows)


def test_clearance_gate_rejects_unachievable_stricter_threshold() -> None:
    profile = PROFILES["ultra"]
    baseline = plan_trajectory(
        profile,
        vision_result=FIXTURE,
        table_clearance_m=0.025,
    )
    strict_clearance = 0.095
    assert baseline.report["minimum_observed_table_distance_m"] < strict_clearance
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
        reports.append(simulation.collision_report(strict_clearance))
    assert any(not report.valid for report in reports)

    with pytest.raises(RuntimeError, match="rejected by collision/edge checks"):
        plan_trajectory(
            profile,
            vision_result=FIXTURE,
            table_clearance_m=strict_clearance,
        )


def test_lift_plan_starts_at_grasp_and_runs_for_two_seconds() -> None:
    approach = plan_trajectory(
        PROFILES["ultra"],
        vision_result=FIXTURE,
        table_clearance_m=0.025,
    )
    lift = plan_lift_trajectory(
        approach,
        lift_height_m=0.045,
        lift_duration_s=2.0,
    )
    assert lift.positions[0] == pytest.approx(approach.positions[-1])
    assert lift.duration_s == pytest.approx(2.0)
    assert lift.report["trajectory_role"] == "lift"
    assert lift.report["verified_collision_free"] is True
    assert lift.report["lift_height_m"] == pytest.approx(0.045)
    assert lift.report["minimum_observed_table_distance_m"] >= 0.025
    assert lift.report["maximum_side_approach_table_tilt_deg"] is None
    assert lift.report["maximum_opening_direction_table_tilt_deg"] is None
