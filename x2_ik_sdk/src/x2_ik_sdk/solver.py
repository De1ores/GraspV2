from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np
import pinocchio as pin

from .config import (
    LEFT_ARM_JOINTS,
    OPTIONAL_WRIST_JOINTS,
    RIGHT_ARM_JOINTS,
    ArmSide,
    X2IKConfig,
)


@dataclass(frozen=True)
class IKResult:
    success: bool
    side: ArmSide
    arm_pos: list[float]
    active_arm: list[float]
    target_xyz: list[float]
    final_xyz: list[float]
    error_norm: float
    iterations: int
    ee_frame: str
    message: str = ""
    target_rpy: list[float] | None = None
    final_rpy: list[float] | None = None


class X2ArmIKSolver:
    def __init__(self, config: X2IKConfig):
        self.config = config
        if not config.urdf_path.exists():
            raise FileNotFoundError(
                "URDF not found: "
                f"{config.urdf_path}\n"
                "The bundled model must exist at "
                "`src/x2_ik_sdk/resources/"
                "x2_ultra_plus_omnipicker_omnipicker.urdf`."
            )
        self.model = pin.buildModelFromUrdf(str(config.urdf_path))
        self._task_frame_ids = {
            side: self._add_tcp_frame(side) for side in (ArmSide.LEFT, ArmSide.RIGHT)
        }
        self.data = self.model.createData()
        self.left_arm_joints = self._resolve_arm_joints(ArmSide.LEFT)
        self.right_arm_joints = self._resolve_arm_joints(ArmSide.RIGHT)
        self.arm_pos_order = self.left_arm_joints + self.right_arm_joints
        self._validate_model()

    @property
    def arm_dof(self) -> int:
        """Degrees of freedom of one arm (5 or 7 for the supported layouts)."""
        n_left = len(self.left_arm_joints)
        n_right = len(self.right_arm_joints)
        if n_left != n_right:
            raise RuntimeError(
                f"Left/right arm DoF mismatch: {n_left} vs {n_right}"
            )
        return n_left

    def solve_position(
        self,
        side: ArmSide | str,
        target_xyz: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> IKResult:
        side = ArmSide(side)
        target = np.asarray(list(target_xyz), dtype=float)
        if target.shape != (3,):
            raise ValueError(f"target_xyz must have length 3, got {target}")

        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_name = self.model.frames[self._task_frame_ids[side]].name
        frame_id = self._task_frame_ids[side]
        active_v_idxs = self._active_velocity_indices(side)

        err_norm = math.inf
        iterations = 0
        success = False
        for iterations in range(1, self.config.max_iters + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[frame_id].translation
            err = target - current
            err_norm = float(np.linalg.norm(err))
            if err_norm < self.config.eps:
                success = True
                break

            jacobian6 = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            jacobian = jacobian6[:3, :]
            q = self._integrate_step(q, jacobian, err, active_v_idxs)

        return self._make_result(
            side=side,
            q=q,
            frame_name=frame_name,
            frame_id=frame_id,
            target_xyz=target.tolist(),
            target_rpy=None,
            err_norm=err_norm,
            iterations=iterations,
            success=success,
        )

    def solve_pose(
        self,
        side: ArmSide | str,
        target_xyz: Iterable[float],
        target_rpy: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> IKResult:
        """Solve IK for end-effector position xyz and orientation rpy."""
        side = ArmSide(side)
        p_des = np.asarray(list(target_xyz), dtype=float)
        rpy_des = np.asarray(list(target_rpy), dtype=float)
        if p_des.shape != (3,):
            raise ValueError(f"target_xyz must have length 3, got {p_des}")
        if rpy_des.shape != (3,):
            raise ValueError(f"target_rpy must have length 3, got {rpy_des}")

        R_des = pin.rpy.rpyToMatrix(float(rpy_des[0]), float(rpy_des[1]), float(rpy_des[2]))
        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_name = self.model.frames[self._task_frame_ids[side]].name
        frame_id = self._task_frame_ids[side]
        active_v_idxs = self._active_velocity_indices(side)

        err_norm = math.inf
        iterations = 0
        success = False
        for iterations in range(1, self.config.max_iters + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            oMf = self.data.oMf[frame_id]

            err_pos = p_des - oMf.translation
            err_rot = pin.log3(R_des @ oMf.rotation.T)
            err = np.concatenate([err_pos, err_rot])
            err_norm = float(np.linalg.norm(err))
            if err_norm < self.config.pose_eps:
                success = True
                break

            jacobian = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            q = self._integrate_step(q, jacobian, err, active_v_idxs)

        return self._make_result(
            side=side,
            q=q,
            frame_name=frame_name,
            frame_id=frame_id,
            target_xyz=p_des.tolist(),
            target_rpy=rpy_des.tolist(),
            err_norm=err_norm,
            iterations=iterations,
            success=success,
        )

    def solve_position_axis(
        self,
        side: ArmSide | str,
        target_xyz: Iterable[float],
        local_axis: Iterable[float],
        target_axis: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
        orientation_weight: float = 0.25,
        position_tolerance_m: float = 1e-3,
        axis_tolerance_rad: float = math.radians(1.0),
    ) -> IKResult:
        """Solve position while aligning one TCP axis, leaving axial yaw free."""

        side = ArmSide(side)
        target = np.asarray(list(target_xyz), dtype=float)
        local = np.asarray(list(local_axis), dtype=float)
        desired = np.asarray(list(target_axis), dtype=float)
        if target.shape != (3,):
            raise ValueError(f"target_xyz must have length 3, got {target}")
        if local.shape != (3,) or desired.shape != (3,):
            raise ValueError("local_axis and target_axis must have length 3")
        if not np.all(np.isfinite(target)):
            raise ValueError("target_xyz must contain finite values")
        local_norm = float(np.linalg.norm(local))
        desired_norm = float(np.linalg.norm(desired))
        if local_norm <= 1e-12 or desired_norm <= 1e-12:
            raise ValueError("local_axis and target_axis must be non-zero")
        if not math.isfinite(orientation_weight) or orientation_weight <= 0.0:
            raise ValueError("orientation_weight must be positive and finite")
        if position_tolerance_m <= 0.0 or axis_tolerance_rad <= 0.0:
            raise ValueError("position and axis tolerances must be positive")
        local /= local_norm
        desired /= desired_norm

        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_name = self.model.frames[self._task_frame_ids[side]].name
        frame_id = self._task_frame_ids[side]
        active_v_idxs = self._active_velocity_indices(side)
        err_norm = math.inf
        iterations = 0
        success = False
        for iterations in range(1, self.config.max_iters + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            oMf = self.data.oMf[frame_id]
            current_axis = oMf.rotation @ local
            err_pos = target - oMf.translation
            err_rot = self._axis_rotation_error(current_axis, desired)
            position_error = float(np.linalg.norm(err_pos))
            axis_error = float(np.linalg.norm(err_rot))
            err = np.concatenate([err_pos, orientation_weight * err_rot])
            err_norm = float(math.hypot(position_error, axis_error))
            if (
                position_error <= position_tolerance_m
                and axis_error <= axis_tolerance_rad
            ):
                success = True
                break
            jacobian = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            weighted_jacobian = jacobian.copy()
            weighted_jacobian[3:, :] *= orientation_weight
            q = self._integrate_step(q, weighted_jacobian, err, active_v_idxs)

        return self._make_result(
            side=side,
            q=q,
            frame_name=frame_name,
            frame_id=frame_id,
            target_xyz=target.tolist(),
            target_rpy=None,
            err_norm=err_norm,
            iterations=iterations,
            success=success,
        )

    @staticmethod
    def _axis_rotation_error(current: np.ndarray, desired: np.ndarray) -> np.ndarray:
        """Return the shortest world-frame rotation taking current to desired."""

        cross = np.cross(current, desired)
        sine = float(np.linalg.norm(cross))
        cosine = float(np.clip(np.dot(current, desired), -1.0, 1.0))
        if sine > 1e-10:
            return cross * (math.atan2(sine, cosine) / sine)
        if cosine >= 0.0:
            return np.zeros(3, dtype=float)
        basis = np.zeros(3, dtype=float)
        basis[int(np.argmin(np.abs(current)))] = 1.0
        perpendicular = np.cross(current, basis)
        perpendicular /= float(np.linalg.norm(perpendicular))
        return math.pi * perpendicular

    def fk_xyz(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        side = ArmSide(side)
        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_id = self._task_frame_ids[side]
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[frame_id].translation.copy().tolist()

    def fk_rpy(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        side = ArmSide(side)
        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_id = self._task_frame_ids[side]
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return pin.rpy.matrixToRpy(self.data.oMf[frame_id].rotation).tolist()

    def q_from_arm_pos(
        self,
        arm_pos: Iterable[float],
        current_head_pos: Iterable[float] | None = None,
    ) -> np.ndarray:
        q = pin.neutral(self.model)
        # Lock optional wrists that exist in URDF but are excluded from arm_pos (5-DoF mode).
        self._apply_locked_optional_wrists(q)
        arm_values = list(arm_pos)
        expected = len(self.arm_pos_order)
        if len(arm_values) != expected:
            raise ValueError(
                f"arm_pos must have length {expected} "
                f"(arm_dof={self.arm_dof} per side), got {len(arm_values)}"
            )
        for joint_name, value in zip(self.arm_pos_order, arm_values):
            self._set_scalar_joint(q, joint_name, value)
        if current_head_pos is not None:
            head = list(current_head_pos)
            if len(head) != 2:
                raise ValueError(f"current_head_pos must have length 2, got {len(head)}")
            self._set_scalar_joint(q, "head_yaw_joint", head[0])
            self._set_scalar_joint(q, "head_pitch_joint", head[1])
        return self._clip_q(q)

    def arm_pos_from_q(self, q: np.ndarray) -> list[float]:
        values = []
        for joint_name in self.arm_pos_order:
            jid = self.model.getJointId(joint_name)
            values.append(float(q[self.model.idx_qs[jid]]))
        return values

    def ready_arm_pos(self) -> list[float]:
        return self._ready_arm_values(ArmSide.LEFT) + self._ready_arm_values(ArmSide.RIGHT)

    def active_joints_for_side(self, side: ArmSide | str) -> list[str]:
        side = ArmSide(side)
        return list(
            self.left_arm_joints if side == ArmSide.LEFT else self.right_arm_joints
        )

    def joint_limits_for_arm_pos(self) -> list[tuple[str, float, float]]:
        limits = []
        for joint_name in self.arm_pos_order:
            jid = self.model.getJointId(joint_name)
            qidx = self.model.idx_qs[jid]
            limits.append(
                (
                    joint_name,
                    float(self.model.lowerPositionLimit[qidx]),
                    float(self.model.upperPositionLimit[qidx]),
                )
            )
        return limits

    def _make_result(
        self,
        *,
        side: ArmSide,
        q: np.ndarray,
        frame_name: str,
        frame_id: int,
        target_xyz: list[float],
        target_rpy: list[float] | None,
        err_norm: float,
        iterations: int,
        success: bool,
    ) -> IKResult:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[frame_id]
        final_xyz = oMf.translation.copy().tolist()
        final_rpy = (
            pin.rpy.matrixToRpy(oMf.rotation).tolist() if target_rpy is not None else None
        )
        arm_pos = self.arm_pos_from_q(q)
        n = self.arm_dof
        active_arm = arm_pos[:n] if side == ArmSide.LEFT else arm_pos[n:]
        msg = "converged" if success else "max iterations reached"
        return IKResult(
            success=success,
            side=side,
            arm_pos=arm_pos,
            active_arm=active_arm,
            target_xyz=target_xyz,
            final_xyz=final_xyz,
            error_norm=err_norm,
            iterations=iterations,
            ee_frame=frame_name,
            message=msg,
            target_rpy=target_rpy,
            final_rpy=final_rpy,
        )

    def _add_tcp_frame(self, side: ArmSide) -> int:
        """Add the configured fixed TCP as an operational frame."""
        parent_name = self.config.frame_for_side(side)
        parent_id = self.model.getFrameId(parent_name)
        if parent_id >= len(self.model.frames):
            raise ValueError(f"URDF is missing end-effector frame: {parent_name}")
        translation, rpy = self.config.tcp_pose_for_side(side)
        translation_array = np.asarray(translation, dtype=float)
        rpy_array = np.asarray(rpy, dtype=float)
        if translation_array.shape != (3,) or not np.all(np.isfinite(translation_array)):
            raise ValueError(f"{side.value} TCP translation must contain three finite values")
        if rpy_array.shape != (3,) or not np.all(np.isfinite(rpy_array)):
            raise ValueError(f"{side.value} TCP rpy must contain three finite values")
        parent = self.model.frames[parent_id]
        parent_to_tcp = pin.SE3(
            pin.rpy.rpyToMatrix(*rpy_array.tolist()),
            translation_array,
        )
        # Frame.placement is relative to parentJoint (parentFrame is metadata),
        # hence compose the end-effector frame placement explicitly.
        placement = parent.placement * parent_to_tcp
        frame_name = f"{parent_name}__graspv2_tcp"
        frame = pin.Frame(
            frame_name,
            parent.parentJoint,
            parent_id,
            placement,
            pin.FrameType.OP_FRAME,
        )
        return int(self.model.addFrame(frame, False))

    def _integrate_step(
        self,
        q: np.ndarray,
        jacobian: np.ndarray,
        err: np.ndarray,
        active_v_idxs: list[int],
    ) -> np.ndarray:
        velocity = self._damped_least_squares(jacobian, err, active_v_idxs)
        step = velocity * self.config.dt
        step_norm = float(np.linalg.norm(step))
        if step_norm > self.config.max_step_norm:
            step *= self.config.max_step_norm / step_norm
        return self._clip_q(pin.integrate(self.model, q, step))

    def _seed_q(
        self,
        current_arm_pos: Iterable[float] | None,
        current_head_pos: Iterable[float] | None,
        q_seed: np.ndarray | None,
    ) -> np.ndarray:
        if q_seed is not None:
            return self._clip_q(np.asarray(q_seed, dtype=float).copy())
        if current_arm_pos is None:
            current_arm_pos = self.ready_arm_pos()
        return self.q_from_arm_pos(current_arm_pos, current_head_pos)

    def _damped_least_squares(
        self,
        jacobian: np.ndarray,
        err: np.ndarray,
        active_v_idxs: list[int],
    ) -> np.ndarray:
        active_jacobian = jacobian[:, active_v_idxs]
        damping = self.config.damping
        active_velocity = active_jacobian.T @ np.linalg.solve(
            active_jacobian @ active_jacobian.T + damping * np.eye(active_jacobian.shape[0]),
            err,
        )
        velocity = np.zeros(self.model.nv)
        velocity[active_v_idxs] = active_velocity
        return velocity

    def _active_velocity_indices(self, side: ArmSide) -> list[int]:
        idxs = []
        for joint_name in self.active_joints_for_side(side):
            jid = self.model.getJointId(joint_name)
            idxs.append(self.model.idx_vs[jid])
        return idxs

    def _clip_q(self, q: np.ndarray) -> np.ndarray:
        margin = self.config.joint_margin
        lower = self.model.lowerPositionLimit + margin
        upper = self.model.upperPositionLimit - margin
        return np.minimum(np.maximum(q, lower), upper)

    def _set_scalar_joint(self, q: np.ndarray, joint_name: str, value: float) -> None:
        if not self.model.existJointName(joint_name):
            raise ValueError(f"URDF is missing joint: {joint_name}")
        jid = self.model.getJointId(joint_name)
        if self.model.joints[jid].nq != 1:
            raise ValueError(f"Joint {joint_name} is not scalar")
        q[self.model.idx_qs[jid]] = float(value)

    def _resolve_arm_joints(self, side: ArmSide) -> list[str]:
        candidates = self.config.candidate_joints_for_side(side)
        explicit = (
            self.config.left_arm_joints
            if side == ArmSide.LEFT
            else self.config.right_arm_joints
        )
        if explicit is not None:
            missing = [name for name in explicit if not self.model.existJointName(name)]
            if missing:
                raise ValueError(
                    f"URDF is missing configured arm joints for {side.value}: {missing}"
                )
            return list(explicit)

        resolved = []
        for joint_name in candidates:
            if self.model.existJointName(joint_name):
                resolved.append(joint_name)
            elif joint_name in OPTIONAL_WRIST_JOINTS:
                continue
            else:
                raise ValueError(f"URDF is missing required arm joint: {joint_name}")
        if len(resolved) not in (5, 7):
            raise ValueError(
                f"Unsupported {side.value} arm DoF={len(resolved)}; "
                "expected 5 (no wrist_pitch/roll) or 7"
            )
        return resolved

    def _ready_arm_values(self, side: ArmSide) -> list[float]:
        joints = self.active_joints_for_side(side)
        ready = self.config.ready_values_for_side(side)
        if len(ready) == len(joints):
            return list(ready)

        candidates = LEFT_ARM_JOINTS if side == ArmSide.LEFT else RIGHT_ARM_JOINTS
        if len(ready) == len(candidates):
            mapping = dict(zip(candidates, ready))
            try:
                return [mapping[name] for name in joints]
            except KeyError as exc:
                raise ValueError(
                    f"ready pose for {side.value} is missing joint {exc.args[0]}"
                ) from exc

        raise ValueError(
            f"ready pose for {side.value} has length {len(ready)}, "
            f"expected {len(joints)} (active) or {len(candidates)} (full 7-DoF template)"
        )

    def _apply_locked_optional_wrists(self, q: np.ndarray) -> None:
        """Keep excluded wrist_pitch/roll at ready values when present in the model."""
        active = set(self.arm_pos_order)
        for side, candidates in (
            (ArmSide.LEFT, LEFT_ARM_JOINTS),
            (ArmSide.RIGHT, RIGHT_ARM_JOINTS),
        ):
            ready = self.config.ready_values_for_side(side)
            if len(ready) == len(candidates):
                template = dict(zip(candidates, ready))
            else:
                # Fall back to the canonical 7-DoF ready template for locked wrists.
                defaults = (
                    [-0.35, 0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
                    if side == ArmSide.LEFT
                    else [-0.35, -0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
                )
                template = dict(zip(candidates, defaults))
            for joint_name in candidates:
                if joint_name in active:
                    continue
                if joint_name not in OPTIONAL_WRIST_JOINTS:
                    continue
                if not self.model.existJointName(joint_name):
                    continue
                self._set_scalar_joint(q, joint_name, template[joint_name])

    def _validate_model(self) -> None:
        if len(self.left_arm_joints) != len(self.right_arm_joints):
            raise ValueError(
                "Left/right arm joint counts must match, got "
                f"{len(self.left_arm_joints)} and {len(self.right_arm_joints)}"
            )
        for frame_name in [self.config.left_ee_frame, self.config.right_ee_frame]:
            if not self.model.existFrame(frame_name):
                raise ValueError(f"URDF is missing expected end-effector frame: {frame_name}")
        for joint_name in self.arm_pos_order:
            jid = self.model.getJointId(joint_name)
            if self.model.joints[jid].nv != 1:
                raise ValueError(f"Expected scalar joint, got {joint_name}")
        # Ensure ready poses are consistent with resolved joints.
        self.ready_arm_pos()
