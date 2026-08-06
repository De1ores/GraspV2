"""Tests for the v0.9 MC ani_path executor arguments."""

from aimdk_msgs.msg import McControlArea

from graspv2.mc_custom_grasp import _parser


def test_ani_path_request_defaults_to_right_arm_without_interrupt() -> None:
    args = _parser().parse_args([])
    assert args.area == McControlArea.RIGHT_HAND
    assert args.motion_id == 9901
    assert args.motion_start_timeout > 0.0
    assert args.return_tolerance > 0.0
    assert args.expected_arm_dof is None


def test_live_profile_can_require_youth_or_ultra_joint_count() -> None:
    assert _parser().parse_args(["--expected-arm-dof", "10"]).expected_arm_dof == 10
    assert _parser().parse_args(["--expected-arm-dof", "14"]).expected_arm_dof == 14
