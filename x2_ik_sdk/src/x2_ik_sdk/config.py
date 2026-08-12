from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path


class ArmSide(str, Enum):
    LEFT = "left"
    RIGHT = "right"


# Full 7-DoF candidates. wrist_pitch / wrist_roll may be absent in 5-DoF models.
LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
]

RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
]

LEFT_ARM_JOINTS_5 = LEFT_ARM_JOINTS[:5]
RIGHT_ARM_JOINTS_5 = RIGHT_ARM_JOINTS[:5]

OPTIONAL_WRIST_JOINTS = (
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
)

ARM_POS_ORDER = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
ARM_POS_ORDER_5 = LEFT_ARM_JOINTS_5 + RIGHT_ARM_JOINTS_5

DEFAULT_URDF_RELATIVE_PATH = (
    Path("resources") / "x2_ultra_plus_omnipicker_omnipicker.urdf"
)


def default_urdf_path() -> Path:
    return Path(__file__).resolve().parent / DEFAULT_URDF_RELATIVE_PATH


@dataclass(frozen=True)
class X2IKConfig:
    urdf_path: Path
    left_ee_frame: str = "L_omnipicker_base_link"
    right_ee_frame: str = "R_omnipicker_base_link"
    # Grasp TCP fixed pose in the corresponding ee_frame.  The solver creates
    # a Pinocchio task frame from these values, so IK and FK both refer to the
    # physical grasp center instead of the OmniPicker mounting-link origin.
    left_tcp_translation_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    left_tcp_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    right_tcp_translation_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    right_tcp_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # None: auto-detect from URDF (skip missing optional wrist joints).
    left_arm_joints: list[str] | None = None
    right_arm_joints: list[str] | None = None
    left_ready_arm: list[float] = field(
        default_factory=lambda: [-0.35, 0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
    )
    right_ready_arm: list[float] = field(
        default_factory=lambda: [-0.35, -0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
    )
    eps: float = 5e-4
    pose_eps: float = 2e-4
    max_iters: int = 1000
    dt: float = 0.1
    damping: float = 1e-4
    max_step_norm: float = 0.05
    joint_margin: float = 1e-6

    @staticmethod
    def default_omnipicker() -> "X2IKConfig":
        return X2IKConfig(urdf_path=default_urdf_path())

    @staticmethod
    def default_omnipicker_5dof() -> "X2IKConfig":
        """Use shoulder/elbow/wrist_yaw only; ignore wrist_pitch and wrist_roll."""
        return X2IKConfig(
            urdf_path=default_urdf_path(),
            left_arm_joints=list(LEFT_ARM_JOINTS_5),
            right_arm_joints=list(RIGHT_ARM_JOINTS_5),
            left_ready_arm=[-0.35, 0.45, 0.0, -1.0, 0.0],
            right_ready_arm=[-0.35, -0.45, 0.0, -1.0, 0.0],
        )

    def frame_for_side(self, side: ArmSide) -> str:
        return self.left_ee_frame if side == ArmSide.LEFT else self.right_ee_frame

    def tcp_pose_for_side(
        self, side: ArmSide
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        if side == ArmSide.LEFT:
            return self.left_tcp_translation_m, self.left_tcp_rpy_rad
        return self.right_tcp_translation_m, self.right_tcp_rpy_rad

    def candidate_joints_for_side(self, side: ArmSide) -> list[str]:
        if side == ArmSide.LEFT:
            return list(self.left_arm_joints or LEFT_ARM_JOINTS)
        return list(self.right_arm_joints or RIGHT_ARM_JOINTS)

    def ready_values_for_side(self, side: ArmSide) -> list[float]:
        return list(self.left_ready_arm if side == ArmSide.LEFT else self.right_ready_arm)
