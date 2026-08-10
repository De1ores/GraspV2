"""Robot variants shared by planning, simulation and playback."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


LEFT_ARM_7 = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
)
RIGHT_ARM_7 = tuple(name.replace("left_", "right_", 1) for name in LEFT_ARM_7)

# The competition robot has one OmniPicker, mounted on the right arm.  Keep
# this topology in the shared profile module so planning, simulation and the
# live hardware boundary cannot silently disagree about the grasping side.
INSTALLED_GRIPPER_SIDE = "right"
AVAILABLE_GRIPPER_SIDES = (INSTALLED_GRIPPER_SIDE,)

# animation_player standing values, ordered as left-7 then right-7. Planning
# starts here so the CSV's lead-in bridge is a collision-neutral hold.
MC_DEFAULT_ARM_POSITION = (
    0.4,
    0.0,
    0.0,
    -1.2,
    0.0,
    0.0,
    0.0,
    0.4,
    0.0,
    0.0,
    -1.2,
    0.0,
    0.0,
    0.0,
)


@dataclass(frozen=True)
class RobotProfile:
    """One verified joint/model contract for the X2 arm pipeline."""

    name: str
    description: str
    left_arm_joints: tuple[str, ...]
    right_arm_joints: tuple[str, ...]
    left_ee_frame: str
    right_ee_frame: str
    base_world_xyz: tuple[float, float, float]
    maximum_velocity_rad_s: float

    @property
    def arm_dof(self) -> int:
        if len(self.left_arm_joints) != len(self.right_arm_joints):
            raise RuntimeError(f"{self.name}: asymmetric arm DoF is unsupported")
        return len(self.left_arm_joints)

    @property
    def arm_pos_order(self) -> tuple[str, ...]:
        return self.left_arm_joints + self.right_arm_joints

    def mc_start_arm_pos(self) -> list[float]:
        """Return physical arm values extracted from animation_player order."""
        by_name = dict(zip(LEFT_ARM_7 + RIGHT_ARM_7, MC_DEFAULT_ARM_POSITION))
        return [by_name[name] for name in self.arm_pos_order]

    def joints_for_side(self, side: str) -> tuple[str, ...]:
        if side == "left":
            return self.left_arm_joints
        if side == "right":
            return self.right_arm_joints
        raise ValueError(f"unsupported arm side: {side!r}")

# arm_sim's stable_stand pelvis height. Planning targets and vision results are
# expressed in that MuJoCo world; the official SDK keeps base_link at the origin.
_STABLE_STAND_BASE_WORLD = (0.0, 0.0, 0.666452127019)


PROFILES: Mapping[str, RobotProfile] = {
    "ultra": RobotProfile(
        name="ultra",
        description="X2 Ultra, seven arm joints per side",
        left_arm_joints=LEFT_ARM_7,
        right_arm_joints=RIGHT_ARM_7,
        left_ee_frame="L_omnipicker_base_link",
        right_ee_frame="R_omnipicker_base_link",
        base_world_xyz=_STABLE_STAND_BASE_WORLD,
        maximum_velocity_rad_s=0.30,
    ),
}


def get_robot_profile(name: str) -> RobotProfile:
    """Resolve a public profile name with a useful error."""
    try:
        return PROFILES[name]
    except KeyError as error:
        available = ", ".join(PROFILES)
        raise ValueError(
            f"unknown robot profile {name!r}; choose one of: {available}"
        ) from error
