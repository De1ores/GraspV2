"""Tests for the compatible MC ani_path executor arguments."""

from types import SimpleNamespace

from aimdk_msgs.msg import JointState, McControlArea
from aimdk_msgs.srv import SetMcPresetMotion
import pytest

from graspv2.mc_custom_grasp import (
    DEFAULT_ARM_POSITION,
    DEFAULT_OMNIPICKER_STUDENT_SDK,
    McCustomGraspClient,
    SafetyError,
    _gripper_event,
    _load_omnipicker_student_sdk,
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
    assert not args.no_gripper
    assert not args.require_gripper_sdk
    assert args.initial_gripper_position == 1.0
    assert args.gripper_events == []


def test_staged_gripper_event_parser_preserves_radius_target() -> None:
    event = _gripper_event("12.500:0.550:close-to-visual-radius")

    assert event.time_s == 12.5
    assert event.position == 0.55
    assert event.label == "close-to-visual-radius"

    with pytest.raises(Exception, match="within"):
        _gripper_event("1.0:1.2:bad")


def test_live_profile_requires_ultra_joint_count() -> None:
    assert _parser().parse_args(["--expected-arm-dof", "14"]).expected_arm_dof == 14


def test_installed_sdk_layout_supports_mc_animation_request() -> None:
    _require_compatible_sdk_layout()
    joint_fields = set(JointState.get_fields_and_field_types())
    assert {"name", "position", "velocity", "effort"} <= joint_fields
    request_fields = set(SetMcPresetMotion.Request.get_fields_and_field_types())
    assert {"header", "area", "motion", "interrupt", "ani_path"} <= request_fields


def test_repository_omnipicker_student_sdk_builds_right_close() -> None:
    sdk = _load_omnipicker_student_sdk(DEFAULT_OMNIPICKER_STUDENT_SDK)
    message = sdk.build_hand_message("right", 0.0)

    assert message.right_hand_type.value == 2
    assert message.right_hands[0].name == "right_claw_joint"
    assert message.right_hands[0].position == 0.0
    assert not message.left_hands


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


def test_packed_legacy_temperature_layout_passes_animation_preflight() -> None:
    packed = [
        (46 << 8) | 49,
        (47 << 8) | 52,
        (51 << 8) | 54,
        (50 << 8) | 54,
    ]
    joints = [
        SimpleNamespace(
            name=name,
            position=position,
            velocity=0.0,
            effort=0.0,
            error_code=(packed[index % len(packed)] if index < 10 else 0),
        )
        for index, (name, position) in enumerate(
            zip(ARM_JOINT_ORDER, DEFAULT_ARM_POSITION)
        )
    ]
    messages = []
    client = SimpleNamespace(
        args=SimpleNamespace(
            interface_timeout=1.0,
            expected_arm_dof=14,
            max_temperature=75,
            max_start_velocity=0.1,
            max_start_error=0.1,
        ),
        arm_state=SimpleNamespace(state=SimpleNamespace(value=0), joints=joints),
        get_logger=lambda: SimpleNamespace(info=messages.append),
    )

    McCustomGraspClient.wait_for_safe_arm_state(client)

    assert any(
        "decoded from legacy packed temperature bytes" in item
        for item in messages
    )


def test_repository_gripper_sdk_failure_is_warning_only() -> None:
    warnings = []

    class BrokenStudentNode:
        def publish_command(self, hand, target_position):
            assert hand == "right"
            assert target_position == 0.0
            raise RuntimeError("control cable unavailable")

    client = SimpleNamespace(
        omnipicker_node=BrokenStudentNode(),
        _warn_gripper=warnings.append,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )

    result = McCustomGraspClient._run_gripper_sdk_best_effort(
        client, "close"
    )

    assert not result
    assert warnings == [
        "student SDK close right failed: control cable unavailable"
    ]


def test_animation_continues_when_repository_gripper_sdk_fails() -> None:
    events = []
    client = SimpleNamespace(
        preflight=lambda: events.append("preflight"),
        _run_gripper_sdk_best_effort=lambda action: events.append(
            f"gripper-failed:{action}"
        ) or False,
        require_sd_mode=lambda: events.append("stable"),
        wait_for_safe_arm_state=lambda: events.append("arm-safe"),
        _arm_positions=lambda: {"joint": 0.0},
        request_animation=lambda: events.append("animation-requested"),
        wait_for_motion_completion=lambda _initial: events.append(
            "animation-complete"
        ),
    )

    McCustomGraspClient.execute(client)

    assert events == [
        "preflight",
        "gripper-failed:open",
        "stable",
        "arm-safe",
        "animation-requested",
        "animation-complete",
    ]


def test_competition_profile_blocks_before_motion_when_sdk_initial_fails() -> None:
    events = []
    client = SimpleNamespace(
        args=SimpleNamespace(
            initial_gripper_position=0.0,
            require_gripper_sdk=True,
        ),
        preflight=lambda: events.append("preflight"),
        _run_gripper_sdk_best_effort=lambda *_args, **_kwargs: False,
        require_sd_mode=lambda: events.append("stable"),
        wait_for_safe_arm_state=lambda: events.append("arm-safe"),
        _arm_positions=lambda: {"joint": 0.0},
        request_animation=lambda: events.append("animation-requested"),
        wait_for_motion_completion=lambda _initial: events.append(
            "animation-complete"
        ),
    )

    with pytest.raises(RuntimeError, match="initial command failed"):
        McCustomGraspClient.execute(client)

    assert events == ["preflight"]
