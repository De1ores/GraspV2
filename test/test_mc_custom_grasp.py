"""Tests for the compatible MC ani_path executor arguments."""

from types import SimpleNamespace

from aimdk_msgs.msg import JointState, McControlArea
from aimdk_msgs.srv import SetMcPresetMotion
import pytest

from graspv2.mc_custom_grasp import (
    DEFAULT_ARM_POSITION,
    McCustomGraspClient,
    SafetyError,
    _parser,
    _require_compatible_sdk_layout,
)
from graspv2.trajectory import ARM_JOINT_ORDER


def test_ani_path_request_defaults_to_right_arm_without_interrupt() -> None:
    args = _parser().parse_args([])
    assert args.area == McControlArea.RIGHT_HAND
    assert args.motion_id == 9901
    assert args.motion_start_timeout > 0.0
    assert args.return_tolerance > 0.0
    assert args.expected_arm_dof == 14


def test_live_profile_requires_ultra_joint_count() -> None:
    assert _parser().parse_args(["--expected-arm-dof", "14"]).expected_arm_dof == 14


def test_installed_sdk_layout_supports_mc_animation_request() -> None:
    _require_compatible_sdk_layout()
    joint_fields = set(JointState.get_fields_and_field_types())
    assert {"name", "position", "velocity", "effort"} <= joint_fields
    request_fields = set(SetMcPresetMotion.Request.get_fields_and_field_types())
    assert {"header", "area", "motion", "interrupt", "ani_path"} <= request_fields


def test_error_code_joint_layout_is_checked_without_temperature_fields() -> None:
    joints = [
        SimpleNamespace(
            name=name,
            position=position,
            velocity=0.0,
            effort=0.0,
            error_code=0,
        )
        for name, position in zip(ARM_JOINT_ORDER, DEFAULT_ARM_POSITION)
    ]
    logger = SimpleNamespace(info=lambda _message: None)
    client = SimpleNamespace(
        args=SimpleNamespace(
            interface_timeout=1.0,
            expected_arm_dof=14,
            max_temperature=75,
            max_start_velocity=0.1,
            max_start_error=0.1,
        ),
        arm_state=SimpleNamespace(state=SimpleNamespace(value=0), joints=joints),
        get_logger=lambda: logger,
    )

    McCustomGraspClient.wait_for_safe_arm_state(client)
    joints[0].error_code = 7
    with pytest.raises(SafetyError, match="non-zero error_code"):
        McCustomGraspClient.wait_for_safe_arm_state(client)
