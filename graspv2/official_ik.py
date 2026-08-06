"""Small adapter around the vendor-provided :mod:`x2_ik_sdk` package."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable

import numpy as np
from x2_ik_sdk import ArmSide, IKResult, X2ArmIKSolver, X2IKConfig

from .robot_profiles import RobotProfile
from .tool_pose import ToolPoseConfig, load_tool_pose_config


@dataclass(frozen=True)
class WorldIKResult:
    """Official IK result with target/final positions in simulation world."""

    sdk_result: IKResult
    target_world_xyz: tuple[float, float, float]
    final_world_xyz: tuple[float, float, float]

    @property
    def success(self) -> bool:
        return self.sdk_result.success

    @property
    def arm_pos(self) -> list[float]:
        return self.sdk_result.arm_pos

    @property
    def active_arm(self) -> list[float]:
        return self.sdk_result.active_arm


class OfficialIK:
    """The only inverse-kinematics backend used by graspV2."""

    def __init__(
        self,
        profile: RobotProfile,
        tool_pose_path: Path | None = None,
    ):
        profile.assert_planning_enabled()
        base_config = (
            X2IKConfig.default_omnipicker_5dof()
            if profile.arm_dof == 5
            else X2IKConfig.default_omnipicker()
        )
        tool_pose = load_tool_pose_config(profile, tool_pose_path)
        config = replace(
            base_config,
            left_tcp_translation_m=tool_pose.left.translation_m,
            left_tcp_rpy_rad=tool_pose.left.rpy_rad,
            right_tcp_translation_m=tool_pose.right.translation_m,
            right_tcp_rpy_rad=tool_pose.right.rpy_rad,
        )
        if tuple(config.candidate_joints_for_side(ArmSide.LEFT)) != (
            profile.left_arm_joints
        ):
            raise RuntimeError(f"{profile.name}: SDK left-arm order mismatch")
        if tuple(config.candidate_joints_for_side(ArmSide.RIGHT)) != (
            profile.right_arm_joints
        ):
            raise RuntimeError(f"{profile.name}: SDK right-arm order mismatch")
        if config.left_ee_frame != profile.left_ee_frame:
            raise RuntimeError(f"{profile.name}: SDK left end-effector mismatch")
        if config.right_ee_frame != profile.right_ee_frame:
            raise RuntimeError(f"{profile.name}: SDK right end-effector mismatch")
        self.profile = profile
        self.tool_pose: ToolPoseConfig = tool_pose
        self.config = config
        self.solver = X2ArmIKSolver(config)
        if tuple(self.solver.arm_pos_order) != profile.arm_pos_order:
            raise RuntimeError(f"{profile.name}: parsed SDK arm order mismatch")

    @property
    def urdf_path(self):
        return self.config.urdf_path

    def ready_arm_pos(self) -> list[float]:
        return self.solver.ready_arm_pos()

    def base_to_world(self, xyz: Iterable[float]) -> tuple[float, float, float]:
        values = np.asarray(tuple(xyz), dtype=float)
        base = np.asarray(self.profile.base_world_xyz, dtype=float)
        if values.shape != (3,):
            raise ValueError("xyz must contain three values")
        return tuple(float(value) for value in values + base)

    def world_to_base(self, xyz: Iterable[float]) -> tuple[float, float, float]:
        values = np.asarray(tuple(xyz), dtype=float)
        base = np.asarray(self.profile.base_world_xyz, dtype=float)
        if values.shape != (3,):
            raise ValueError("xyz must contain three values")
        return tuple(float(value) for value in values - base)

    def fk_world(self, side: str, arm_pos: Iterable[float]) -> tuple[float, float, float]:
        base_xyz = self.solver.fk_xyz(ArmSide(side), arm_pos)
        return self.base_to_world(base_xyz)

    def fk_rpy(self, side: str, arm_pos: Iterable[float]) -> tuple[float, float, float]:
        return tuple(
            float(value)
            for value in self.solver.fk_rpy(ArmSide(side), arm_pos)
        )

    def solve_world_position(
        self,
        side: str,
        target_world_xyz: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
    ) -> WorldIKResult:
        target_world = tuple(float(value) for value in target_world_xyz)
        result = self.solver.solve_position(
            ArmSide(side),
            self.world_to_base(target_world),
            current_arm_pos=current_arm_pos,
        )
        return WorldIKResult(
            sdk_result=result,
            target_world_xyz=target_world,
            final_world_xyz=self.base_to_world(result.final_xyz),
        )

    def joint_limits_for_side(
        self,
        side: str,
    ) -> tuple[np.ndarray, np.ndarray]:
        selected = set(self.profile.joints_for_side(side))
        limits = {
            name: (lower, upper)
            for name, lower, upper in self.solver.joint_limits_for_arm_pos()
            if name in selected
        }
        ordered = [limits[name] for name in self.profile.joints_for_side(side)]
        return (
            np.asarray([value[0] for value in ordered], dtype=float),
            np.asarray([value[1] for value in ordered], dtype=float),
        )
