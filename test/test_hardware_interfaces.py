"""Tests for the ROS-independent AimDK hardware boundary."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from graspv2.aimdk_hardware import (
    _load_ros_types,
    build_joint_message,
    build_omnipicker_message,
    build_upper_body_message,
    create_aimdk_hardware_node,
    require_aimdk_control_schema,
)
from graspv2.hardware_contract import (
    HardwareContractError,
    UpperBodyFrame,
    arm_setpoints,
    load_hardware_config,
    resolve_joint_feedback,
    upper_body_arm_positions,
)
from graspv2.grasp_sequence import GraspPlanMetadata, VisualCheckResult
from graspv2.robot_profiles import PROFILES
from graspv2.trajectory import JointTrajectory


def _joint(name: str, position: float):
    return SimpleNamespace(name=name, position=position)


def test_default_config_uses_x2_aimdk_topics() -> None:
    config = load_hardware_config()
    assert config.topics.arm_command == "/aima/hal/joint/arm/command"
    assert config.topics.hand_state == "/aima/hal/joint/hand/state"
    assert config.topics.upper_body_command == "/mc/upper_body_command"
    assert config.services.set_mc_action.endswith("/SetMcAction")
    assert config.omnipicker.left_joint_name == "left_claw_joint"
    assert config.omnipicker.require_feedback is False
    assert config.topics.rgb_image.endswith("/rgbd_head_front/rgb_image")
    assert config.topics.depth_camera_info.endswith("/depth_camera_info")


def test_feedback_accepts_documented_order_or_complete_names() -> None:
    expected = ("a", "b")
    unnamed = (_joint("", 1.0), _joint("", 2.0))
    assert resolve_joint_feedback(unnamed, expected) == (1.0, 2.0)

    named_reversed = (_joint("b", 2.0), _joint("a", 1.0))
    assert resolve_joint_feedback(named_reversed, expected) == (1.0, 2.0)

    with pytest.raises(HardwareContractError, match="partially populated"):
        resolve_joint_feedback((_joint("a", 1.0), _joint("", 2.0)), expected)


def test_arm_setpoints_overlay_active_side_and_hold_other_side() -> None:
    profile = PROFILES["youth"]
    base = [0.1 * index for index in range(profile.arm_dof * 2)]
    active = {name: 0.25 for name in profile.right_arm_joints}
    velocities = {name: -0.05 for name in profile.right_arm_joints}
    commands = arm_setpoints(
        profile,
        base,
        active,
        velocities,
        load_hardware_config().upper_body,
    )
    assert tuple(command.name for command in commands) == profile.arm_pos_order
    assert [command.position for command in commands[: profile.arm_dof]] == base[
        : profile.arm_dof
    ]
    assert all(
        command.position == 0.25 and command.velocity == -0.05
        for command in commands[profile.arm_dof :]
    )


def test_upper_body_layout_expands_youth_missing_wrists() -> None:
    profile = PROFILES["youth"]
    expanded = upper_body_arm_positions(profile, tuple(range(10)))
    assert expanded == (
        0.0,
        1.0,
        2.0,
        3.0,
        4.0,
        0.0,
        0.0,
        5.0,
        6.0,
        7.0,
        8.0,
        9.0,
        0.0,
        0.0,
    )


def test_ros_builders_match_installed_aimdk_contract() -> None:
    types = _load_ros_types()
    require_aimdk_control_schema(types)
    config = load_hardware_config()
    profile = PROFILES["youth"]
    commands = arm_setpoints(
        profile,
        [0.0] * 10,
        {profile.left_arm_joints[0]: 0.2},
        {profile.left_arm_joints[0]: 0.1},
        config.upper_body,
    )
    joint_message = build_joint_message(types, commands)
    assert len(joint_message.joints) == 10
    assert joint_message.joints[0].stiffness == 20.0

    hand_message = build_omnipicker_message(types, config, "right", 0.0)
    assert hand_message.left_hand_type.value == 0
    assert hand_message.left_hands == []
    assert hand_message.right_hand_type.value == 2
    assert len(hand_message.right_hands) == 1
    assert hand_message.right_hands[0].name == "right_claw_joint"
    assert hand_message.right_hands[0].position == 0.0

    upper_message = build_upper_body_message(
        types,
        config,
        UpperBodyFrame(
            head_positions=(0.1, -0.1),
            arm_positions=tuple(0.01 * index for index in range(14)),
            hand_sub_mode=1,
            hand_positions=(1.0, 0.0),
        ),
        sequence=7,
    )
    assert upper_message.header.sequence == 7
    assert upper_message.source == "remote_teleop_pc"
    assert len(upper_message.arm_pos) == 14
    assert upper_message.hand_sub_mode == 1
    assert list(upper_message.hand_pos) == [1.0, 0.0]


def test_competition_hand_publisher_uses_reference_qos() -> None:
    class FakeQoS:
        def __init__(self, **values):
            vars(self).update(values)

    class FakePublisher:
        def __init__(self, qos_profile):
            self.qos_profile = qos_profile

        def publish(self, _message) -> None:
            pass

        def get_subscription_count(self) -> int:
            return 0

    class FakeNode:
        # rclpy.Node already owns this read-only property; the adapter must not
        # attempt to assign an instance attribute with the same name.
        @property
        def publishers(self):
            return ()

        def __init__(self, _name):
            self.created_publishers = {}

        def create_publisher(self, _message_type, topic, qos_profile):
            publisher = FakePublisher(qos_profile)
            self.created_publishers[topic] = publisher
            return publisher

        def create_subscription(self, *_args):
            return object()

        def create_client(self, *_args):
            return object()

    original = _load_ros_types()
    types = replace(original, Node=FakeNode, QoSProfile=FakeQoS)
    config = load_hardware_config()
    node = create_aimdk_hardware_node(types, config)
    publisher = node.command_publishers["hand"]
    assert publisher.qos_profile.depth == 10
    assert (
        publisher.qos_profile.reliability
        == types.ReliabilityPolicy.BEST_EFFORT
    )
    assert (
        publisher.qos_profile.durability
        == types.DurabilityPolicy.TRANSIENT_LOCAL
    )


def test_visual_grasp_sequence_orders_hand_vision_lift_and_recovery() -> None:
    class FakeQoS:
        def __init__(self, **values):
            vars(self).update(values)

    class FakePublisher:
        def publish(self, _message) -> None:
            pass

        def get_subscription_count(self) -> int:
            return 1

    class FakeNode:
        @property
        def publishers(self):
            return ()

        def __init__(self, _name):
            pass

        def create_publisher(self, *_args):
            return FakePublisher()

        def create_subscription(self, *_args):
            return object()

        def create_client(self, *_args):
            return object()

    original = _load_ros_types()
    types = replace(original, Node=FakeNode, QoSProfile=FakeQoS)
    node = create_aimdk_hardware_node(types, load_hardware_config())
    profile = PROFILES["youth"]
    names = profile.right_arm_joints
    approach = JointTrajectory(
        source=Path("approach.json"),
        joint_names=names,
        times=(0.0, 1.0),
        positions=(tuple(0.0 for _ in names), tuple(0.1 for _ in names)),
        maximum_velocity=0.1,
    )
    lift = JointTrajectory(
        source=Path("lift.json"),
        joint_names=names,
        times=(0.0, 2.0),
        positions=(tuple(0.1 for _ in names), tuple(0.2 for _ in names)),
        maximum_velocity=0.05,
    )
    metadata = GraspPlanMetadata(
        robot_profile="youth",
        side="right",
        grasp_target_world_m=(0.4, -0.2, 0.8),
        lifted_target_world_m=(0.4, -0.2, 0.9),
        lift_direction_world=(0.0, 0.0, 1.0),
        lift_height_m=0.1,
        lift_duration_s=2.0,
    )
    events = []
    node._wait_for_mode_services = lambda: events.append("mode-services")
    node._wait_for_command_consumer = lambda group: events.append(f"consumer:{group}")
    node._assert_arm_health = lambda *_args, **_kwargs: (0.0,) * 10
    node._assert_head_health = lambda: (0.0, 0.0)
    node._validate_upper_body_start = lambda *_args: None
    node._require_stable_mode = lambda: events.append("stable")
    node.execute_omnipicker = lambda side, position: events.append(
        f"preopen:{side}:{position:.1f}"
    )
    node._enter_split_mode = lambda: events.append("enter-split")
    node.restore_stable_mode = lambda: events.append("restore-stable")

    def run_segment(_profile, _trajectory, _base, _head, *, label):
        events.append(f"trajectory:{label}")
        return SimpleNamespace(label=label)

    node._run_upper_body_trajectory = run_segment
    node._command_omnipicker_while_holding = (
        lambda _profile, _frame, side, position: events.append(
            f"hand:{side}:{position:.1f}"
        )
    )

    def verify_hold(_profile, _frame, verifier, *, timeout_s, label):
        events.append(f"vision:{label}:{timeout_s:.0f}")
        return verifier()

    node._verify_while_holding = verify_hold

    def check(stage: str) -> VisualCheckResult:
        return VisualCheckResult(
            stage=stage,
            passed=True,
            observed_world_m=(0.4, -0.2, 0.8),
            expected_world_m=(0.4, -0.2, 0.8),
            target_error_m=0.0,
        )

    node.execute_grasp_sequence(
        profile,
        approach,
        lift,
        metadata,
        lambda: check("closed_grasp"),
        lambda: check("post_lift"),
        verification_timeout_s=45.0,
    )
    assert events == [
        "mode-services",
        "consumer:hand",
        "stable",
        "preopen:right:1.0",
        "enter-split",
        "consumer:upper-body",
        "trajectory:approach",
        "hand:right:0.0",
        "vision:closed-grasp:45",
        "trajectory:two-second lift",
        "vision:post-lift:45",
        "trajectory:lower",
        "hand:right:1.0",
        "trajectory:retreat",
        "restore-stable",
    ]
