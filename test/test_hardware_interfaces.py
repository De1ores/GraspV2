"""Tests for the ROS-independent AimDK hardware boundary."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

import graspv2.aimdk_hardware as hardware
from graspv2.aimdk_hardware import (
    UpperBodyUnavailableError,
    _animation_fallback_eligible,
    _competition_omnipicker_sdk_argv,
    _largest_tracking_error,
    _load_ros_types,
    _wait_for_control_services,
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
    inspect_joint_health,
    load_hardware_config,
    require_installed_omnipicker_side,
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
    assert config.schema_version == 2
    assert config.topics.arm_command == "/aima/hal/joint/arm/command"
    assert config.topics.hand_state == "/aima/hal/joint/hand/state"
    assert config.topics.upper_body_command == "/mc/upper_body_command"
    assert config.services.set_mc_action.endswith("/SetMcAction")
    assert config.services.get_current_input_source.endswith(
        "/GetCurrentInputSource"
    )
    assert config.services.set_mc_input_source.endswith("/SetMcInputSource")
    assert config.upper_body.command_source == "graspv2"
    assert config.upper_body.command_source_priority == 65
    assert config.upper_body.service_discovery_timeout_s == 15.0
    assert config.omnipicker.installed_side == "right"
    assert config.omnipicker.require_feedback is False
    assert config.topics.rgb_image.endswith("/rgbd_head_front/rgb_image")
    assert config.topics.depth_camera_info.endswith("/depth_camera_info")
    require_installed_omnipicker_side(config, "right")
    with pytest.raises(HardwareContractError, match="installed only on right"):
        require_installed_omnipicker_side(config, "left")


def test_feedback_accepts_documented_order_or_complete_names() -> None:
    expected = ("a", "b")
    unnamed = (_joint("", 1.0), _joint("", 2.0))
    assert resolve_joint_feedback(unnamed, expected) == (1.0, 2.0)

    named_reversed = (_joint("b", 2.0), _joint("a", 1.0))
    assert resolve_joint_feedback(named_reversed, expected) == (1.0, 2.0)

    with pytest.raises(HardwareContractError, match="partially populated"):
        resolve_joint_feedback((_joint("a", 1.0), _joint("", 2.0)), expected)


def test_mixed_overlay_packed_temperatures_are_not_joint_errors() -> None:
    temperatures = ((46, 49), (47, 52), (51, 54), (50, 54))
    joints = [
        SimpleNamespace(
            name=f"joint_{index}",
            error_code=(coil << 8) | motor,
        )
        for index, (coil, motor) in enumerate(temperatures)
    ]
    health = inspect_joint_health(joints)
    assert health.packed_legacy_temperatures
    assert health.hottest_temperature_c == 54
    assert not health.error_codes

    single = inspect_joint_health(
        [SimpleNamespace(name="shoulder", error_code=(46 << 8) | 49)]
    )
    assert single.error_codes == (("shoulder", 11825),)


def test_arm_setpoints_overlay_active_side_and_hold_other_side() -> None:
    profile = PROFILES["ultra"]
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


def test_upper_body_layout_preserves_ultra_joint_order() -> None:
    profile = PROFILES["ultra"]
    positions = tuple(range(14))
    assert upper_body_arm_positions(profile, positions) == pytest.approx(positions)


def test_omnipicker_builder_commands_only_the_installed_right_side() -> None:
    class HandType:
        def __init__(self, *, value: int):
            self.value = value

    class HandCommand:
        pass

    class HandCommandArray:
        def __init__(self):
            self.left_hands = []
            self.right_hands = []

    class MessageHeader:
        def __init__(self):
            self.frame_id = ""

    types = SimpleNamespace(
        HandType=HandType,
        HandCommand=HandCommand,
        HandCommandArray=HandCommandArray,
        MessageHeader=MessageHeader,
    )
    config = load_hardware_config()
    message = build_omnipicker_message(types, config, "right", 0.0)
    assert message.left_hand_type.value == 0
    assert message.left_hands == []
    assert message.right_hand_type.value == 2
    assert len(message.right_hands) == 1
    assert message.right_hands[0].name == "right_claw_joint"
    assert message.header.frame_id == "hand_command"
    with pytest.raises(HardwareContractError, match="installed only on right"):
        build_omnipicker_message(types, config, "left", 0.0)


def test_ros_builders_match_installed_aimdk_contract() -> None:
    types = _load_ros_types()
    require_aimdk_control_schema(types)
    config = load_hardware_config()
    profile = PROFILES["ultra"]
    commands = arm_setpoints(
        profile,
        [0.0] * 14,
        {profile.left_arm_joints[0]: 0.2},
        {profile.left_arm_joints[0]: 0.1},
        config.upper_body,
    )
    joint_message = build_joint_message(types, commands)
    assert len(joint_message.joints) == 14
    assert joint_message.joints[0].stiffness == 20.0

    hand_message = build_omnipicker_message(types, config, "right", 0.0)
    assert hand_message.left_hand_type.value == 0
    assert hand_message.left_hands == []
    assert hand_message.right_hand_type.value == 2
    assert len(hand_message.right_hands) == 1
    assert hand_message.right_hands[0].name == "right_claw_joint"
    assert hand_message.right_hands[0].position == 0.0
    assert hand_message.header.frame_id == "hand_command"

    frame = UpperBodyFrame(
        head_positions=(0.1, -0.1),
        arm_positions=tuple(0.01 * index for index in range(14)),
        hand_sub_mode=1,
        hand_positions=(1.0, 0.0),
    )
    if types.UpperBodyCommandArray is None:
        with pytest.raises(UpperBodyUnavailableError):
            build_upper_body_message(types, config, frame, sequence=7)
    else:
        upper_message = build_upper_body_message(
            types, config, frame, sequence=7
        )
        assert upper_message.header.sequence == 7
        assert upper_message.source == "graspv2"
        assert len(upper_message.arm_pos) == 14
        assert upper_message.hand_sub_mode == 1
        assert list(upper_message.hand_pos) == [1.0, 0.0]


def test_largest_tracking_error_reports_joint_and_values() -> None:
    feedback = (0.0, -0.01, 0.20)
    index_by_name = {"shoulder": 0, "elbow": 1, "wrist": 2}
    targets = {"shoulder": -0.355, "elbow": 0.02}

    error, name, actual, target = _largest_tracking_error(
        feedback,
        index_by_name,
        targets,
    )

    assert error == pytest.approx(0.355)
    assert name == "shoulder"
    assert actual == pytest.approx(0.0)
    assert target == pytest.approx(-0.355)


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


def test_each_control_service_gets_a_full_discovery_timeout() -> None:
    class FakeClient:
        def __init__(self):
            self.timeouts = []

        def wait_for_service(self, *, timeout_sec):
            self.timeouts.append(timeout_sec)
            return True

    config = load_hardware_config()
    clients = [FakeClient() for _ in range(4)]

    _wait_for_control_services(
        tuple((client, f"/service/{index}") for index, client in enumerate(clients)),
        config.upper_body.service_discovery_timeout_s,
    )

    assert [client.timeouts for client in clients] == [
        [config.upper_body.service_discovery_timeout_s]
    ] * 4


def test_visual_grasp_sequence_orders_hand_vision_lift_and_recovery() -> None:
    class FakeUpperBodyCommandArray:
        @staticmethod
        def get_fields_and_field_types():
            return {
                name: "unused"
                for name in (
                    "header",
                    "source",
                    "hand_sub_mode",
                    "head_pos",
                    "arm_pos",
                    "hand_pos",
                )
            }

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
    types = replace(
        original,
        Node=FakeNode,
        QoSProfile=FakeQoS,
        UpperBodyCommandArray=FakeUpperBodyCommandArray,
    )
    node = create_aimdk_hardware_node(types, load_hardware_config())
    profile = PROFILES["ultra"]
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
    return_to_default = JointTrajectory(
        source=Path("return.json"),
        joint_names=names,
        times=(0.0, 2.0, 3.0),
        positions=(
            tuple(0.2 for _ in names),
            tuple(0.1 for _ in names),
            tuple(profile.mc_start_arm_pos()[profile.arm_dof :]),
        ),
        maximum_velocity=1.3,
    )
    metadata = GraspPlanMetadata(
        robot_profile="ultra",
        side="right",
        object_center_world_m=(0.4, -0.2, 0.795),
        gripper_center_world_m=(0.4, -0.2, 0.8),
        lifted_object_center_world_m=(0.4, -0.2, 0.895),
        lifted_gripper_center_world_m=(0.4, -0.2, 0.9),
        lift_direction_world=(0.0, 0.0, 1.0),
        lift_height_m=0.1,
        lift_duration_s=2.0,
        pregrasp_duration_s=0.5,
        visual_radius_m=0.045,
        preopen_position=1.0,
        grip_position=0.72,
        open_duration_s=0.5,
        close_duration_s=0.8,
        grasp_settle_duration_s=0.3,
        lifted_hold_duration_s=2.5,
        controlled_lower_duration_s=2.0,
        open_hand_retreat_duration_s=0.5,
        release_duration_s=0.6,
        place_settle_duration_s=0.7,
        reclose_duration_s=0.6,
    )
    events = []
    node._wait_for_mode_services = lambda: events.append("mode-services")
    node._wait_for_command_consumer = lambda group: events.append(f"consumer:{group}")
    node._assert_arm_health = lambda *_args, **_kwargs: (0.0,) * 14
    node._assert_head_health = lambda: (0.0, 0.0)
    node._validate_upper_body_start = lambda *_args: None
    node._require_stable_mode = lambda: events.append("stable")
    node.configure_input_source = lambda: events.append("configure-source")
    node.execute_omnipicker = lambda side, position: events.append(
        f"preopen:{side}:{position:.1f}"
    )
    node._enter_split_mode = lambda: events.append("enter-split")
    node._activate_input_source = (
        lambda *_args: events.append("activate-source")
    )
    node.restore_stable_mode = lambda: events.append("restore-stable")

    def run_segment(_profile, _trajectory, _base, _head, *, label):
        node.planned_motion_started = True
        events.append(f"trajectory:{label}")
        return SimpleNamespace(label=label)

    node._run_upper_body_trajectory = run_segment
    node._command_omnipicker_while_holding = (
        lambda _profile, _frame, side, position, **_kwargs: events.append(
            f"hand:{side}:{position:.1f}"
        )
    )
    node._hold_upper_body_pose = (
        lambda _profile, _frame, duration, *, label: events.append(
            f"hold:{label}:{duration:.1f}"
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
        return_to_default,
        metadata,
        lambda: check("closed_grasp"),
        lambda: check("post_lift"),
        verification_timeout_s=45.0,
    )
    assert node.planned_motion_started
    assert events == [
        "mode-services",
        "consumer:hand",
        "stable",
        "configure-source",
        "enter-split",
        "consumer:upper-body",
        "activate-source",
        "preopen:right:0.0",
        "trajectory:move-above-object",
        "hand:right:1.0",
        "trajectory:vertical-descent",
        "hand:right:0.7",
        "hold:grasp-settle:0.3",
        "vision:closed-grasp:45",
        "trajectory:two-second lift",
        "vision:post-lift:45",
        "hold:lifted-hold:2.5",
        "trajectory:controlled-lower",
        "hand:right:1.0",
        "hold:place-settle:0.7",
        "trajectory:open-hand-vertical-retreat",
        "hand:right:0.0",
        "trajectory:return-to-default",
        "restore-stable",
    ]


def test_animation_fallback_is_blocked_after_first_upper_body_command() -> None:
    args = SimpleNamespace(
        operation="trajectory",
        transport="upper-body",
        upper_body_fallback="animation",
    )
    animation = Path("fallback.csv")
    assert _animation_fallback_eligible(
        args,
        SimpleNamespace(upper_body_command_count=0),
        animation,
    )
    assert not _animation_fallback_eligible(
        args,
        SimpleNamespace(upper_body_command_count=1),
        animation,
    )
    args.operation = "grasp"
    assert _animation_fallback_eligible(
        args,
        SimpleNamespace(upper_body_command_count=0),
        animation,
    )
    assert not _animation_fallback_eligible(
        args,
        SimpleNamespace(
            upper_body_command_count=0,
            omnipicker_command_count=1,
        ),
        animation,
    )


def test_setup_hold_and_initial_hand_frames_can_fallback_before_planned_motion(
) -> None:
    args = SimpleNamespace(
        operation="grasp",
        upper_body_fallback="animation",
    )
    animation = Path("fallback.csv")
    node = SimpleNamespace(
        planned_motion_started=False,
        upper_body_command_count=100,
        omnipicker_command_count=100,
    )

    assert _animation_fallback_eligible(args, node, animation)

    node.planned_motion_started = True
    assert not _animation_fallback_eligible(args, node, animation)


def test_competition_omnipicker_uses_repository_sdk_with_phase_duration() -> None:
    sdk = Path("/home/agi/graspV2/omnipicker_hand_student.py")

    assert _competition_omnipicker_sdk_argv(
        sdk,
        "right",
        0.725,
        0.8,
    ) == (
        "/usr/bin/python3",
        str(sdk),
        "--publish",
        "--duration",
        "0.800000000",
        "position",
        "right",
        "0.725000000",
    )


def test_competition_grasp_can_fallback_if_upper_body_init_has_no_node(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("GRASPV2_RUNTIME_PROFILE", "competition")
    args = SimpleNamespace(
        operation="grasp",
        upper_body_fallback="animation",
    )

    assert _animation_fallback_eligible(args, None, Path("fallback.csv"))


def test_main_starts_prepared_animation_after_pre_motion_failure(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    class FakeRclpy:
        active = False

        @classmethod
        def init(cls, *, args):
            assert args == []
            cls.active = True

        @classmethod
        def ok(cls):
            return cls.active

        @classmethod
        def shutdown(cls):
            cls.active = False

    class FakeLogger:
        def warning(self, _message):
            pass

        def error(self, _message):
            pass

    class FakeNode:
        upper_body_command_count = 0
        destroyed = False

        def execute_upper_body_trajectory(self, _profile, _trajectory):
            # Vendor bindings also surface non-RuntimeError failures. The main
            # boundary must route any ordinary pre-motion failure to animation.
            raise OSError("vendor upper-body transport unavailable")

        def get_logger(self):
            return FakeLogger()

        def destroy_node(self):
            self.destroyed = True

    trajectory = SimpleNamespace(
        frame_count=2,
        duration=1.0,
        maximum_velocity=0.1,
    )
    fallback = tmp_path / "fallback.csv"
    fake_node = FakeNode()
    calls = []
    monkeypatch.setattr(
        hardware,
        "_load_ros_types",
        lambda: SimpleNamespace(rclpy=FakeRclpy),
    )
    monkeypatch.setattr(hardware, "configure_fastdds_logging", lambda: None)
    monkeypatch.setattr(
        hardware,
        "create_aimdk_hardware_node",
        lambda _types, _config: fake_node,
    )
    monkeypatch.setattr(
        hardware,
        "load_trajectory",
        lambda *_args, **_kwargs: trajectory,
    )
    monkeypatch.setattr(
        hardware,
        "validate_animation_trajectory_source",
        lambda *_args: None,
    )
    monkeypatch.setattr(
        hardware,
        "build_mc_animation",
        lambda *_args, **_kwargs: object(),
    )
    monkeypatch.setattr(
        hardware,
        "write_mc_animation_csv",
        lambda _animation, _path: fallback,
    )
    monkeypatch.setattr(
        hardware,
        "validate_mc_animation_csv",
        lambda *_args, **_kwargs: SimpleNamespace(frame_count=2, duration_s=1.0),
    )

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(hardware.subprocess, "run", run)
    result = hardware.main(
        [
            "trajectory",
            "--trajectory",
            str(tmp_path / "trajectory.json"),
            "--execute",
            "--confirm-control-authority",
        ]
    )

    assert result == 0
    assert fake_node.destroyed
    assert not FakeRclpy.active
    assert calls[0][0][-3:] == ["--animation", str(fallback), "--yes"]
