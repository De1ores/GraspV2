"""Collision-checked arm planning driven exclusively by the official IK SDK."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time
from typing import Iterable

import numpy as np

from .official_ik import OfficialIK, WorldIKResult
from .robot_profiles import (
    AVAILABLE_GRIPPER_SIDES,
    INSTALLED_GRIPPER_SIDE,
    RobotProfile,
)
from .simulation import (
    CollisionReport,
    RobotSimulation,
    TableObstacle,
    load_table_obstacle,
    resolve_robot_visual_urdf,
    validate_fk_alignment,
)


@dataclass
class TreeNode:
    joints: np.ndarray
    parent: int | None


@dataclass(frozen=True)
class PlannedTrajectory:
    profile: RobotProfile
    side: str
    obstacle: TableObstacle | None
    visual_urdf_path: Path | None
    target_world_xyz: tuple[float, float, float]
    pregrasp_world_xyz: tuple[float, float, float]
    joint_names: tuple[str, ...]
    times: tuple[float, ...]
    positions: tuple[tuple[float, ...], ...]
    report: dict[str, object]

    @property
    def duration_s(self) -> float:
        return self.times[-1]

    def document(self) -> dict[str, object]:
        frames = []
        for index, (time_s, positions) in enumerate(zip(self.times, self.positions)):
            frames.append(
                {
                    "frame": index,
                    "time_s": time_s,
                    "active_joints_rad": dict(zip(self.joint_names, positions)),
                }
            )
        return {
            "format": "x2_active_joint_trajectory",
            "format_version": 1,
            "units": {"time": "s", "joint_angle": "rad"},
            "robot_profile": self.profile.name,
            "arm_side": self.side,
            "active_joint_names": list(self.joint_names),
            "frame_count": len(frames),
            "duration_s": self.duration_s,
            "frames": frames,
            "planning": self.report,
        }

    def write(self, trajectory_path: Path, report_path: Path) -> None:
        trajectory_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        trajectory_path.write_text(
            json.dumps(self.document(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        report_path.write_text(
            json.dumps(self.report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


class CollisionChecker:
    """State and edge validation over one active arm."""

    def __init__(
        self,
        simulation: RobotSimulation,
        side: str,
        base_arm_pos: Iterable[float],
        clearance_m: float,
        edge_resolution_rad: float,
    ):
        self.simulation = simulation
        self.side = side
        self.base_arm_pos = tuple(float(value) for value in base_arm_pos)
        self.clearance_m = float(clearance_m)
        self.edge_resolution_rad = float(edge_resolution_rad)
        self.state_checks = 0
        self.minimum_table_distance_m = simulation.probe_distance_m
        self.nearest_table_body: str | None = None
        self.last_invalid_report: CollisionReport | None = None

    def arm_pos_for(self, joints: Iterable[float]) -> list[float]:
        arm_pos = list(self.base_arm_pos)
        values = list(float(value) for value in joints)
        offset = 0 if self.side == "left" else self.simulation.profile.arm_dof
        arm_pos[offset:offset + self.simulation.profile.arm_dof] = values
        return arm_pos

    def state_report(self, joints: Iterable[float]) -> CollisionReport:
        self.state_checks += 1
        self.simulation.set_arm_pos(self.arm_pos_for(joints))
        report = self.simulation.collision_report(self.clearance_m)
        if report.minimum_table_distance_m < self.minimum_table_distance_m:
            self.minimum_table_distance_m = report.minimum_table_distance_m
            self.nearest_table_body = report.nearest_table_body
        if not report.valid:
            self.last_invalid_report = report
        return report

    def state_valid(self, joints: Iterable[float]) -> bool:
        return self.state_report(joints).valid

    def edge_valid(self, start: Iterable[float], goal: Iterable[float]) -> bool:
        start_values = np.asarray(tuple(start), dtype=float)
        goal_values = np.asarray(tuple(goal), dtype=float)
        delta = goal_values - start_values
        samples = max(
            1,
            int(
                math.ceil(
                    float(np.max(np.abs(delta))) / self.edge_resolution_rad
                )
            ),
        )
        return all(
            self.state_valid(start_values + phase * delta)
            for phase in np.linspace(0.0, 1.0, samples + 1)
        )


def _reconstruct(nodes: list[TreeNode], index: int) -> list[np.ndarray]:
    result = []
    while index is not None:
        node = nodes[index]
        result.append(node.joints.copy())
        index = node.parent
    result.reverse()
    return result


def plan_rrt(
    start: np.ndarray,
    goal: np.ndarray,
    checker: CollisionChecker,
    lower: np.ndarray,
    upper: np.ndarray,
    rng: np.random.Generator,
    *,
    step_size_rad: float = 0.24,
    maximum_iterations: int = 2500,
    goal_bias: float = 0.22,
) -> tuple[list[np.ndarray], dict[str, object]]:
    """Deterministic goal-biased RRT with collision checks on every edge."""
    if not checker.state_valid(start):
        raise RuntimeError(f"start pose is in collision: {checker.last_invalid_report}")
    if not checker.state_valid(goal):
        raise RuntimeError(f"pregrasp pose is in collision: {checker.last_invalid_report}")
    if checker.edge_valid(start, goal):
        return [start.copy(), goal.copy()], {
            "direct_path": True,
            "iterations": 0,
            "nodes": 2,
        }

    scale = upper - lower
    nodes = [TreeNode(start.copy(), None)]
    for iteration in range(1, maximum_iterations + 1):
        sample = goal if rng.random() < goal_bias else rng.uniform(lower, upper)
        nearest_index = min(
            range(len(nodes)),
            key=lambda candidate: float(
                np.linalg.norm((nodes[candidate].joints - sample) / scale)
            ),
        )
        nearest = nodes[nearest_index].joints
        delta = sample - nearest
        distance = float(np.linalg.norm(delta))
        candidate = (
            sample.copy()
            if distance <= step_size_rad
            else nearest + delta * (step_size_rad / distance)
        )
        if not checker.edge_valid(nearest, candidate):
            continue
        nodes.append(TreeNode(candidate, nearest_index))
        new_index = len(nodes) - 1
        if checker.edge_valid(candidate, goal):
            nodes.append(TreeNode(goal.copy(), new_index))
            return _reconstruct(nodes, len(nodes) - 1), {
                "direct_path": False,
                "iterations": iteration,
                "nodes": len(nodes),
            }
    raise RuntimeError(
        f"RRT failed after {maximum_iterations} iterations and {len(nodes)} nodes"
    )


def shortcut_path(
    path: list[np.ndarray],
    checker: CollisionChecker,
    rng: np.random.Generator,
    attempts: int = 120,
) -> list[np.ndarray]:
    result = [point.copy() for point in path]
    for _ in range(attempts):
        if len(result) <= 2:
            break
        first, second = sorted(rng.choice(len(result), size=2, replace=False))
        if second <= first + 1:
            continue
        if checker.edge_valid(result[first], result[second]):
            result[first + 1:second] = []
    return result


def _random_arm_seed(
    ik: OfficialIK,
    side: str,
    base_arm_pos: list[float],
    rng: np.random.Generator,
) -> list[float]:
    lower, upper = ik.joint_limits_for_side(side)
    result = list(base_arm_pos)
    offset = 0 if side == "left" else ik.profile.arm_dof
    result[offset:offset + ik.profile.arm_dof] = rng.uniform(lower, upper)
    return result


def solve_collision_free_ik(
    ik: OfficialIK,
    checker: CollisionChecker,
    side: str,
    target_world_xyz: Iterable[float],
    preferred_arm_pos: list[float],
    rng: np.random.Generator,
    *,
    attempts: int = 14,
    required_edge_start: np.ndarray | None = None,
) -> WorldIKResult:
    """Try official IK from deterministic seeds and reject colliding results."""
    last_result: WorldIKResult | None = None
    target = tuple(float(value) for value in target_world_xyz)
    ik_solution_count = 0
    for attempt in range(attempts):
        seed = (
            preferred_arm_pos
            if attempt == 0
            else _random_arm_seed(ik, side, preferred_arm_pos, rng)
        )
        result = ik.solve_world_position(side, target, seed)
        last_result = result
        if not result.success:
            continue
        ik_solution_count += 1
        active = np.asarray(result.active_arm, dtype=float)
        if not checker.state_valid(active):
            continue
        if required_edge_start is not None and not checker.edge_valid(
            required_edge_start, active
        ):
            continue
        return result
    suffix = ""
    if last_result is not None:
        suffix = (
            f"; final SDK error={last_result.sdk_result.error_norm:.6g}, "
            f"message={last_result.sdk_result.message}"
        )
    target_text = "[" + ", ".join(f"{value:.6f}" for value in target) + "]"
    if ik_solution_count == 0:
        raise RuntimeError(
            f"official IK found no solution for {side} target_world_m={target_text} "
            f"after {attempts} attempts{suffix}"
        )
    raise RuntimeError(
        f"official IK solutions for {side} target_world_m={target_text} were rejected "
        f"by collision/edge checks after {attempts} attempts{suffix}"
    )


def _minimum_jerk(ratio: float) -> float:
    return ratio**3 * (10.0 + ratio * (-15.0 + 6.0 * ratio))


def sample_waypoints(
    waypoints: list[np.ndarray],
    maximum_velocity_rad_s: float,
    *,
    rate_hz: float = 50.0,
) -> tuple[tuple[float, ...], tuple[tuple[float, ...], ...]]:
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    times = [0.0]
    positions = [tuple(float(value) for value in waypoints[0])]
    elapsed = 0.0
    for start, goal in zip(waypoints, waypoints[1:]):
        delta = goal - start
        # The peak derivative of minimum-jerk is 1.875 times displacement/duration.
        duration = max(
            0.24,
            1.875 * float(np.max(np.abs(delta))) / maximum_velocity_rad_s,
        )
        samples = max(2, int(math.ceil(duration * rate_hz)))
        duration = samples / rate_hz
        for index in range(1, samples + 1):
            ratio = index / samples
            blend = _minimum_jerk(ratio)
            elapsed += 1.0 / rate_hz
            times.append(round(elapsed, 9))
            positions.append(
                tuple(float(value) for value in start + blend * delta)
            )
    return tuple(times), tuple(positions)


def sample_waypoints_fixed_duration(
    waypoints: list[np.ndarray],
    duration_s: float,
    *,
    rate_hz: float = 50.0,
) -> tuple[tuple[float, ...], tuple[tuple[float, ...], ...]]:
    """Sample a piecewise minimum-jerk path over one exact, aligned duration."""

    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    if duration_s <= 0.0 or rate_hz <= 0.0:
        raise ValueError("duration_s and rate_hz must be positive")
    ticks = int(round(duration_s * rate_hz))
    if ticks < len(waypoints) - 1:
        raise ValueError("duration is too short to sample every waypoint segment")
    aligned_duration = ticks / rate_hz
    if abs(aligned_duration - duration_s) > 1e-9:
        raise ValueError(
            f"duration_s must align to the {rate_hz:.1f} Hz command period"
        )
    segment_weights = np.asarray(
        [
            max(float(np.max(np.abs(goal - start))), 1e-9)
            for start, goal in zip(waypoints, waypoints[1:])
        ],
        dtype=float,
    )
    cumulative = np.concatenate(([0.0], np.cumsum(segment_weights)))
    total = float(cumulative[-1])
    times = []
    positions = []
    for tick in range(ticks + 1):
        time_s = tick / rate_hz
        progress = total * tick / ticks
        segment = min(
            len(segment_weights) - 1,
            max(0, int(np.searchsorted(cumulative, progress, side="right") - 1)),
        )
        local = (progress - cumulative[segment]) / segment_weights[segment]
        blend = _minimum_jerk(max(0.0, min(1.0, float(local))))
        position = waypoints[segment] + blend * (
            waypoints[segment + 1] - waypoints[segment]
        )
        times.append(round(time_s, 9))
        positions.append(tuple(float(value) for value in position))
    positions[0] = tuple(float(value) for value in waypoints[0])
    positions[-1] = tuple(float(value) for value in waypoints[-1])
    return tuple(times), tuple(positions)


def measured_maximum_velocity(
    times: Iterable[float], positions: Iterable[Iterable[float]]
) -> float:
    time_values = tuple(times)
    position_values = tuple(tuple(row) for row in positions)
    maximum = 0.0
    for index in range(1, len(time_values)):
        dt = time_values[index] - time_values[index - 1]
        maximum = max(
            maximum,
            max(
                abs(current - previous) / dt
                for previous, current in zip(
                    position_values[index - 1], position_values[index]
                )
            ),
        )
    return maximum


def _default_target(ik: OfficialIK, side: str, ready: list[float]) -> tuple[float, float, float]:
    current = np.asarray(ik.fk_world(side, ready), dtype=float)
    lateral = 0.035 if side == "right" else -0.035
    return tuple(float(value) for value in current + (0.055, lateral, 0.025))


def plan_trajectory(
    profile: RobotProfile,
    *,
    side: str = "right",
    target_world_xyz: Iterable[float] | None = None,
    vision_result: Path | None = None,
    robot_urdf: Path | None = None,
    table_clearance_m: float = 0.025,
    approach_distance_m: float = 0.075,
    random_seed: int = 11,
) -> PlannedTrajectory:
    """Plan and densely verify one approach trajectory in MuJoCo."""
    started = time.monotonic()
    if side not in AVAILABLE_GRIPPER_SIDES:
        raise ValueError(
            "grasp side must be right because the competition robot has no "
            "left OmniPicker"
        )
    if not 0.0 <= table_clearance_m <= 0.10:
        raise ValueError("table_clearance_m must be within [0, 0.10]")
    if not 0.01 <= approach_distance_m <= 0.30:
        raise ValueError("approach_distance_m must be within [0.01, 0.30]")

    obstacle: TableObstacle | None = None
    vision_target: tuple[float, float, float] | None = None
    if vision_result is not None:
        vision_target, obstacle = load_table_obstacle(vision_result)
    visual_urdf_path = resolve_robot_visual_urdf(robot_urdf)
    ik = OfficialIK(profile)
    start_arm_pos = profile.mc_start_arm_pos()
    target = tuple(
        float(value)
        for value in (
            target_world_xyz
            if target_world_xyz is not None
            else vision_target
            if vision_target is not None
            else _default_target(ik, side, start_arm_pos)
        )
    )
    if len(target) != 3 or not all(math.isfinite(value) for value in target):
        raise ValueError("target_world_xyz must contain three finite values")
    approach_normal = np.asarray(
        obstacle.plane_normal if obstacle is not None else (0.0, 0.0, 1.0),
        dtype=float,
    )
    pregrasp = tuple(
        float(value)
        for value in np.asarray(target, dtype=float)
        + approach_distance_m * approach_normal
    )

    alignment = validate_fk_alignment(profile, ik, random_samples=4, seed=random_seed)
    if alignment.maximum_position_error_m > 1e-3:
        raise RuntimeError(
            f"SDK/MuJoCo FK position mismatch: {alignment.maximum_position_error_m:.6g} m"
        )
    if alignment.maximum_orientation_error_deg > 0.5:
        raise RuntimeError(
            "SDK/MuJoCo FK orientation mismatch: "
            f"{alignment.maximum_orientation_error_deg:.6g} deg"
        )

    simulation = RobotSimulation(
        profile,
        ik,
        obstacle,
        visual_urdf_path=visual_urdf_path,
    )
    checker = CollisionChecker(
        simulation,
        side,
        start_arm_pos,
        table_clearance_m if obstacle is not None else 0.0,
        edge_resolution_rad=0.01,
    )
    rng = np.random.default_rng(random_seed)
    offset = 0 if side == "left" else profile.arm_dof
    start_joints = np.asarray(
        start_arm_pos[offset:offset + profile.arm_dof], dtype=float
    )

    pregrasp_result = solve_collision_free_ik(
        ik, checker, side, pregrasp, start_arm_pos, rng
    )
    pregrasp_joints = np.asarray(pregrasp_result.active_arm, dtype=float)
    lower, upper = ik.joint_limits_for_side(side)
    path, rrt_report = plan_rrt(
        start_joints,
        pregrasp_joints,
        checker,
        lower,
        upper,
        rng,
    )
    path = shortcut_path(path, checker, rng)

    approach_segments = max(4, int(math.ceil(approach_distance_m / 0.015)))
    previous_joints = pregrasp_joints
    previous_arm_pos = pregrasp_result.arm_pos
    final_result = pregrasp_result
    for index in range(1, approach_segments + 1):
        ratio = index / approach_segments
        cartesian_target = (
            np.asarray(pregrasp, dtype=float)
            + ratio * (np.asarray(target, dtype=float) - np.asarray(pregrasp, dtype=float))
        )
        final_result = solve_collision_free_ik(
            ik,
            checker,
            side,
            cartesian_target,
            previous_arm_pos,
            rng,
            attempts=8,
            required_edge_start=previous_joints,
        )
        previous_joints = np.asarray(final_result.active_arm, dtype=float)
        previous_arm_pos = final_result.arm_pos
        path.append(previous_joints.copy())

    times, positions = sample_waypoints(path, profile.maximum_velocity_rad_s)
    maximum_velocity = measured_maximum_velocity(times, positions)
    if maximum_velocity > profile.maximum_velocity_rad_s + 1e-6:
        raise RuntimeError(
            f"sampled trajectory velocity {maximum_velocity:.6g} exceeds profile limit"
        )
    verification_checker = CollisionChecker(
        simulation,
        side,
        start_arm_pos,
        table_clearance_m if obstacle is not None else 0.0,
        edge_resolution_rad=0.01,
    )
    for position in positions:
        if not verification_checker.state_valid(position):
            raise RuntimeError(
                "densely sampled trajectory is in collision: "
                f"{verification_checker.last_invalid_report}"
            )

    final_error = float(
        np.linalg.norm(
            np.asarray(final_result.final_world_xyz) - np.asarray(target)
        )
    )
    if final_error > 1e-3:
        raise RuntimeError(f"final position error {final_error:.6g} m exceeds 1 mm")
    report: dict[str, object] = {
        "verified_collision_free": True,
        "trajectory_role": "approach",
        "robot_profile": profile.name,
        "arm_side": side,
        "installed_gripper_side": INSTALLED_GRIPPER_SIDE,
        "ik_backend": "x2_ik_sdk.X2ArmIKSolver",
        "kinematic_urdf": str(ik.urdf_path),
        "tool_pose_calibration": str(ik.tool_pose.source_path),
        "active_tcp_offset": {
            "parent_frame": ik.tool_pose.for_side(side).parent_frame,
            "translation_m": list(ik.tool_pose.for_side(side).translation_m),
            "rpy_rad": list(ik.tool_pose.for_side(side).rpy_rad),
        },
        "robot_visual_urdf": (
            str(visual_urdf_path) if visual_urdf_path is not None else None
        ),
        "omnipicker_visual_description": (
            str(simulation.omnipicker_description_path)
            if simulation.omnipicker_description_path is not None
            else None
        ),
        "start_pose": "mc_animation_default",
        "target_world_m": list(target),
        "pregrasp_world_m": list(pregrasp),
        "final_position_error_m": final_error,
        "table_clearance_m": table_clearance_m if obstacle is not None else None,
        "minimum_observed_table_distance_m": (
            verification_checker.minimum_table_distance_m
            if obstacle is not None
            else None
        ),
        "nearest_table_body": verification_checker.nearest_table_body,
        "state_checks": checker.state_checks + verification_checker.state_checks,
        "rrt": {
            **rrt_report,
            "shortcut_waypoints": len(path) - approach_segments,
        },
        "approach_segments": approach_segments,
        "frame_count": len(times),
        "duration_s": times[-1],
        "maximum_velocity_rad_s": maximum_velocity,
        "fk_alignment": {
            "samples": alignment.samples,
            "maximum_position_error_m": alignment.maximum_position_error_m,
            "maximum_orientation_error_deg": alignment.maximum_orientation_error_deg,
        },
        "vision_result": str(vision_result.resolve()) if vision_result else None,
        "table_obstacle": (
            {
                "center_m": list(obstacle.center_m),
                "quaternion_wxyz": list(obstacle.quaternion_wxyz),
                "half_extents_m": list(obstacle.half_extents_m),
                "plane_normal": list(obstacle.plane_normal),
            }
            if obstacle is not None
            else None
        ),
        "planning_time_s": time.monotonic() - started,
    }
    return PlannedTrajectory(
        profile=profile,
        side=side,
        obstacle=obstacle,
        visual_urdf_path=visual_urdf_path,
        target_world_xyz=target,
        pregrasp_world_xyz=pregrasp,
        joint_names=profile.joints_for_side(side),
        times=times,
        positions=positions,
        report=report,
    )


def plan_lift_trajectory(
    approach: PlannedTrajectory,
    *,
    lift_height_m: float = 0.10,
    lift_duration_s: float = 2.0,
    random_seed: int = 23,
) -> PlannedTrajectory:
    """Plan a collision-checked Cartesian lift from an approach endpoint."""

    if approach.side not in AVAILABLE_GRIPPER_SIDES:
        raise ValueError(
            "lift side must be right because the competition robot has no "
            "left OmniPicker"
        )
    if not 0.03 <= lift_height_m <= 0.30:
        raise ValueError("lift_height_m must be within [0.03, 0.30]")
    if not 0.5 <= lift_duration_s <= 10.0:
        raise ValueError("lift_duration_s must be within [0.5, 10.0]")
    profile = approach.profile
    started = time.monotonic()
    ik = OfficialIK(profile)
    simulation = RobotSimulation(
        profile,
        ik,
        approach.obstacle,
        visual_urdf_path=approach.visual_urdf_path,
    )
    table_clearance = approach.report.get("table_clearance_m")
    checker = CollisionChecker(
        simulation,
        approach.side,
        profile.mc_start_arm_pos(),
        float(table_clearance) if table_clearance is not None else 0.0,
        edge_resolution_rad=0.01,
    )
    direction = np.asarray(
        (
            approach.obstacle.plane_normal
            if approach.obstacle is not None
            else (0.0, 0.0, 1.0)
        ),
        dtype=float,
    )
    direction /= float(np.linalg.norm(direction))
    if direction[2] < 0.0:
        direction *= -1.0
    start_target = np.asarray(approach.target_world_xyz, dtype=float)
    lifted_target = start_target + lift_height_m * direction
    start_joints = np.asarray(approach.positions[-1], dtype=float)
    if not checker.state_valid(start_joints):
        raise RuntimeError(
            "approach endpoint is invalid for lift planning: "
            f"{checker.last_invalid_report}"
        )
    preferred_arm_pos = checker.arm_pos_for(start_joints)
    previous_joints = start_joints
    path = [start_joints.copy()]
    rng = np.random.default_rng(random_seed)
    segments = max(4, int(math.ceil(lift_height_m / 0.015)))
    final_result: WorldIKResult | None = None
    for index in range(1, segments + 1):
        target = start_target + (index / segments) * lift_height_m * direction
        final_result = solve_collision_free_ik(
            ik,
            checker,
            approach.side,
            target,
            preferred_arm_pos,
            rng,
            attempts=10,
            required_edge_start=previous_joints,
        )
        previous_joints = np.asarray(final_result.active_arm, dtype=float)
        preferred_arm_pos = final_result.arm_pos
        path.append(previous_joints.copy())
    assert final_result is not None

    times, positions = sample_waypoints_fixed_duration(
        path,
        lift_duration_s,
    )
    maximum_velocity = measured_maximum_velocity(times, positions)
    if maximum_velocity > profile.maximum_velocity_rad_s + 1e-6:
        raise RuntimeError(
            f"two-second lift requires {maximum_velocity:.3f} rad/s, above "
            f"the {profile.maximum_velocity_rad_s:.3f} rad/s profile limit"
        )
    verification = CollisionChecker(
        simulation,
        approach.side,
        profile.mc_start_arm_pos(),
        float(table_clearance) if table_clearance is not None else 0.0,
        edge_resolution_rad=0.01,
    )
    for previous, current in zip(positions, positions[1:]):
        if not verification.edge_valid(previous, current):
            raise RuntimeError(
                "densely sampled lift trajectory is in collision: "
                f"{verification.last_invalid_report}"
            )
    final_error = float(
        np.linalg.norm(
            np.asarray(final_result.final_world_xyz) - lifted_target
        )
    )
    if final_error > 1e-3:
        raise RuntimeError(f"lift final position error {final_error:.6g} m exceeds 1 mm")
    report: dict[str, object] = {
        "verified_collision_free": True,
        "trajectory_role": "lift",
        "robot_profile": profile.name,
        "arm_side": approach.side,
        "installed_gripper_side": INSTALLED_GRIPPER_SIDE,
        "ik_backend": "x2_ik_sdk.X2ArmIKSolver",
        "kinematic_urdf": str(ik.urdf_path),
        "tool_pose_calibration": str(ik.tool_pose.source_path),
        "robot_visual_urdf": (
            str(approach.visual_urdf_path)
            if approach.visual_urdf_path is not None
            else None
        ),
        "omnipicker_visual_description": (
            str(simulation.omnipicker_description_path)
            if simulation.omnipicker_description_path is not None
            else None
        ),
        "start_pose": "grasp_endpoint",
        "lift_start_world_m": list(approach.target_world_xyz),
        "target_world_m": [float(value) for value in lifted_target],
        "lift_direction_world": [float(value) for value in direction],
        "lift_height_m": lift_height_m,
        "lift_duration_s": times[-1],
        "final_position_error_m": final_error,
        "table_clearance_m": table_clearance,
        "minimum_observed_table_distance_m": (
            verification.minimum_table_distance_m
            if approach.obstacle is not None
            else None
        ),
        "nearest_table_body": verification.nearest_table_body,
        "state_checks": checker.state_checks + verification.state_checks,
        "cartesian_segments": segments,
        "frame_count": len(times),
        "duration_s": times[-1],
        "maximum_velocity_rad_s": maximum_velocity,
        "source_approach_vision_result": approach.report.get("vision_result"),
        "planning_time_s": time.monotonic() - started,
    }
    return PlannedTrajectory(
        profile=profile,
        side=approach.side,
        obstacle=approach.obstacle,
        visual_urdf_path=approach.visual_urdf_path,
        target_world_xyz=tuple(float(value) for value in lifted_target),
        pregrasp_world_xyz=approach.target_world_xyz,
        joint_names=approach.joint_names,
        times=times,
        positions=positions,
        report=report,
    )


def replay_viewer(trajectory: PlannedTrajectory) -> None:
    """Replay a verified trajectory in the MuJoCo passive viewer."""
    try:
        from mujoco import viewer
    except ImportError as error:
        raise RuntimeError(f"MuJoCo viewer is unavailable: {error}") from error
    ik = OfficialIK(trajectory.profile)
    simulation = RobotSimulation(
        trajectory.profile,
        ik,
        trajectory.obstacle,
        visual_urdf_path=trajectory.visual_urdf_path,
    )
    start_arm_pos = trajectory.profile.mc_start_arm_pos()
    threads_before = set(threading.enumerate())
    window = viewer.launch_passive(simulation.model, simulation.data)
    viewer_threads = set(threading.enumerate()) - threads_before
    try:
        window.cam.lookat[:] = (0.25, 0.0, 0.75)
        window.cam.distance = 2.15
        window.cam.azimuth = 200.0
        window.cam.elevation = -20.0
        for position in trajectory.positions:
            if not window.is_running():
                break
            simulation.set_side_joints(trajectory.side, position, start_arm_pos)
            window.sync()
            time.sleep(0.02)
    finally:
        window.close()
        # launch_passive uses a daemon UI thread on Linux.  Waiting for its
        # GLFW/X11 teardown prevents the interpreter from unloading native
        # libraries underneath that thread and crashing on process exit.
        for thread in viewer_threads:
            thread.join(timeout=5.0)
        still_running = [thread.name for thread in viewer_threads if thread.is_alive()]
        if still_running:
            raise RuntimeError(
                "MuJoCo Viewer did not shut down cleanly: " + ", ".join(still_running)
            )


def preview_scene(
    profile: RobotProfile,
    *,
    vision_result: Path | None,
    robot_urdf: Path | None = None,
) -> None:
    """Open an interactive static scene when planning has no valid trajectory."""
    try:
        from mujoco import viewer
    except ImportError as error:
        raise RuntimeError(f"MuJoCo viewer is unavailable: {error}") from error
    obstacle = (
        load_table_obstacle(vision_result)[1]
        if vision_result is not None
        else None
    )
    ik = OfficialIK(profile)
    simulation = RobotSimulation(
        profile,
        ik,
        obstacle,
        visual_urdf_path=resolve_robot_visual_urdf(robot_urdf),
    )
    threads_before = set(threading.enumerate())
    window = viewer.launch_passive(simulation.model, simulation.data)
    viewer_threads = set(threading.enumerate()) - threads_before
    try:
        window.cam.lookat[:] = (0.25, 0.0, 0.75)
        window.cam.distance = 2.15
        window.cam.azimuth = 200.0
        window.cam.elevation = -20.0
        while window.is_running():
            window.sync()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        window.close()
        for thread in viewer_threads:
            thread.join(timeout=5.0)
        still_running = [thread.name for thread in viewer_threads if thread.is_alive()]
        if still_running:
            raise RuntimeError(
                "MuJoCo Viewer did not shut down cleanly: " + ", ".join(still_running)
            )
