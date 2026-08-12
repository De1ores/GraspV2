"""Live X2 AimDK adapters for MC upper-body, HAL joints and OmniPicker.

Nothing in this module is contacted by the planner.  The command-line entry
point is dry-run by default and direct HAL publication requires both
``--execute`` and ``--confirm-control-authority``.  Arm trajectories default
to MC upper-body split mode so balance control remains owned by MC; direct HAL
publication is an explicit fallback transport.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Callable, Mapping, Sequence

from .grasp_sequence import (
    GraspPlanMetadata,
    VisualCheckResult,
    VisualObservation,
    VisualVerificationConfig,
    VisualVerificationError,
    load_grasp_plan_metadata,
    load_visual_observation,
    validate_trajectory_continuity,
    verify_closed_observation,
    verify_initial_observation,
    verify_lifted_observation,
    write_grasp_status,
)

from .hardware_contract import (
    DEFAULT_HARDWARE_CONFIG_PATH,
    HardwareConfig,
    HardwareContractError,
    JointSetpoint,
    UpperBodyFrame,
    arm_setpoints,
    inspect_joint_health,
    load_hardware_config,
    require_installed_omnipicker_side,
    resolve_joint_feedback,
    trajectory_sample_velocity,
    upper_body_arm_positions,
)
from .mc_animation import (
    build_mc_animation,
    build_mc_grasp_animation,
    McGripperEvent,
    validate_animation_trajectory_source,
    validate_mc_animation_csv,
    write_mc_animation_csv,
)
from .robot_profiles import (
    AVAILABLE_GRIPPER_SIDES,
    INSTALLED_GRIPPER_SIDE,
    PROFILES,
    RobotProfile,
    get_robot_profile,
)
from .ros_logging import configure_fastdds_logging
from .trajectory import (
    JointTrajectory,
    load_trajectory,
    reverse_trajectory,
    slice_trajectory,
)


class LiveHardwareError(RuntimeError):
    """Raised when live feedback or an AimDK interface is unsafe to use."""


class UpperBodyUnavailableError(LiveHardwareError):
    """Raised before motion when the selected SDK has no upper-body schema."""


def _competition_omnipicker_sdk_argv(
    sdk: Path,
    side: str,
    position: float,
    duration_s: float,
) -> tuple[str, ...]:
    """Build the one official repository-SDK command used on competition X2."""

    return (
        "/usr/bin/python3",
        str(sdk),
        "--publish",
        "--duration",
        f"{duration_s:.9f}",
        "position",
        side,
        f"{position:.9f}",
    )


@dataclass(frozen=True)
class _RosTypes:
    rclpy: object
    Node: type
    QoSProfile: type
    ReliabilityPolicy: object
    HistoryPolicy: object
    DurabilityPolicy: object
    JointCommand: type
    JointCommandArray: type
    JointStateArray: type
    HandCommand: type
    HandCommandArray: type
    HandStateArray: type
    HandType: type
    UpperBodyCommandArray: type | None
    MessageHeader: type
    CommonRequest: type
    RequestHeader: type
    McActionCommand: type
    McInputAction: type
    McInputSource: type
    GetMcAction: type
    SetMcAction: type
    GetCurrentInputSource: type
    SetMcInputSource: type


def _load_ros_types() -> _RosTypes:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from aimdk_msgs.msg import (
            HandCommand,
            HandCommandArray,
            HandStateArray,
            HandType,
            JointCommand,
            JointCommandArray,
            JointStateArray,
            McActionCommand,
            McInputAction,
            McInputSource,
            MessageHeader,
            CommonRequest,
            RequestHeader,
        )
        from aimdk_msgs.srv import (
            GetCurrentInputSource,
            GetMcAction,
            SetMcAction,
            SetMcInputSource,
        )
    except (ImportError, ModuleNotFoundError) as error:
        raise LiveHardwareError(
            "ROS 2/AimDK Python interfaces are unavailable. Source "
            "/opt/ros/humble/setup.bash and the matching AimDK overlay first. "
            f"Import error: {error}"
        ) from error
    try:
        from aimdk_msgs.msg import UpperBodyCommandArray
    except (ImportError, ModuleNotFoundError):
        # The official AimDK v1.0.0 package removed this legacy MC split-mode
        # message while retaining HAL, hand and preset-motion interfaces.
        UpperBodyCommandArray = None
    return _RosTypes(
        rclpy=rclpy,
        Node=Node,
        QoSProfile=QoSProfile,
        ReliabilityPolicy=ReliabilityPolicy,
        HistoryPolicy=HistoryPolicy,
        DurabilityPolicy=DurabilityPolicy,
        JointCommand=JointCommand,
        JointCommandArray=JointCommandArray,
        JointStateArray=JointStateArray,
        HandCommand=HandCommand,
        HandCommandArray=HandCommandArray,
        HandStateArray=HandStateArray,
        HandType=HandType,
        UpperBodyCommandArray=UpperBodyCommandArray,
        MessageHeader=MessageHeader,
        CommonRequest=CommonRequest,
        RequestHeader=RequestHeader,
        McActionCommand=McActionCommand,
        McInputAction=McInputAction,
        McInputSource=McInputSource,
        GetMcAction=GetMcAction,
        SetMcAction=SetMcAction,
        GetCurrentInputSource=GetCurrentInputSource,
        SetMcInputSource=SetMcInputSource,
    )


def _fields(message_type: type) -> set[str]:
    return set(message_type.get_fields_and_field_types())


def require_aimdk_control_schema(
    types: _RosTypes,
    *,
    require_upper_body: bool = False,
) -> None:
    """Validate common AimDK interfaces and, when requested, legacy split mode."""

    expected = {
        types.JointCommand: {
            "name",
            "position",
            "velocity",
            "effort",
            "stiffness",
            "damping",
        },
        types.JointCommandArray: {"header", "joints"},
        types.HandCommand: {
            "name",
            "position",
            "velocity",
            "acceleration",
            "deceleration",
            "effort",
        },
        types.HandCommandArray: {
            "header",
            "left_hand_type",
            "left_hands",
            "right_hand_type",
            "right_hands",
        },
        types.HandStateArray: {
            "header",
            "left_hand_type",
            "left_hands",
            "left_touch_sensors",
            "right_hand_type",
            "right_hands",
            "right_touch_sensors",
        },
        types.McActionCommand: {"action", "action_desc"},
        types.McInputAction: {"value"},
        types.McInputSource: {"name", "priority", "timeout"},
    }
    for message_type, required in expected.items():
        actual = _fields(message_type)
        if actual != required:
            raise LiveHardwareError(
                f"AimDK {message_type.__name__} schema mismatch: "
                f"received {sorted(actual)}, expected {sorted(required)}"
            )
    if types.UpperBodyCommandArray is None:
        if require_upper_body:
            raise UpperBodyUnavailableError(
                "this AimDK overlay does not provide UpperBodyCommandArray; "
                "official v1.0.0 supports the MC animation fallback instead"
            )
    else:
        required_upper_body = {
            "header",
            "source",
            "hand_sub_mode",
            "head_pos",
            "arm_pos",
            "hand_pos",
        }
        actual = _fields(types.UpperBodyCommandArray)
        if actual != required_upper_body:
            raise LiveHardwareError(
                "AimDK UpperBodyCommandArray schema mismatch: "
                f"received {sorted(actual)}, expected "
                f"{sorted(required_upper_body)}"
            )
    joint_state_array = _fields(types.JointStateArray)
    if joint_state_array != {"header", "state", "joints"}:
        raise LiveHardwareError(
            "AimDK JointStateArray schema mismatch: "
            f"received {sorted(joint_state_array)}"
        )
    service_schemas = {
        types.GetMcAction.Request: {"request"},
        types.GetMcAction.Response: {"header", "info"},
        types.SetMcAction.Request: {"header", "source", "command"},
        types.SetMcAction.Response: {"response"},
        types.GetCurrentInputSource.Request: {"request"},
        types.GetCurrentInputSource.Response: {"response", "input_source"},
        types.SetMcInputSource.Request: {"request", "action", "input_source"},
        types.SetMcInputSource.Response: {"response"},
    }
    for message_type, required in service_schemas.items():
        actual = _fields(message_type)
        if actual != required:
            raise LiveHardwareError(
                f"AimDK {message_type.__name__} schema mismatch: "
                f"received {sorted(actual)}, expected {sorted(required)}"
            )


def build_joint_message(types: _RosTypes, commands: Sequence[JointSetpoint]):
    """Convert ROS-independent setpoints to the official AimDK message."""

    if not commands:
        raise HardwareContractError("a joint command group cannot be empty")
    message = types.JointCommandArray()
    for setpoint in commands:
        joint = types.JointCommand()
        joint.name = setpoint.name
        joint.position = float(setpoint.position)
        joint.velocity = float(setpoint.velocity)
        joint.effort = float(setpoint.effort)
        joint.stiffness = float(setpoint.stiffness)
        joint.damping = float(setpoint.damping)
        message.joints.append(joint)
    return message


def build_omnipicker_message(
    types: _RosTypes,
    config: HardwareConfig,
    side: str,
    position: float,
):
    """Build one selected-side OmniPicker command; the other side is untouched."""

    if side not in {"left", "right"}:
        raise HardwareContractError("OmniPicker side must be left or right")
    require_installed_omnipicker_side(config, side)
    if not 0.0 <= position <= 1.0:
        raise HardwareContractError("OmniPicker position must be within [0, 1]")
    tuning = config.omnipicker
    command = types.HandCommand()
    command.name = tuning.right_joint_name
    command.position = float(position)
    command.velocity = tuning.velocity
    command.acceleration = tuning.acceleration
    command.deceleration = tuning.deceleration
    command.effort = tuning.effort

    message = types.HandCommandArray()
    message.header = types.MessageHeader()
    message.header.frame_id = "hand_command"
    # 0 means no command/device for this message; 2 means OmniPicker/gripper.
    message.left_hand_type = types.HandType(value=0)
    message.right_hand_type = types.HandType(value=0)
    message.right_hand_type = types.HandType(value=2)
    message.right_hands = [command]
    message.left_hands = []
    return message


def build_upper_body_message(
    types: _RosTypes,
    config: HardwareConfig,
    frame: UpperBodyFrame,
    sequence: int,
    stamp: object | None = None,
):
    """Build the official fixed-width MC upper-body split command."""

    if types.UpperBodyCommandArray is None:
        raise UpperBodyUnavailableError(
            "UpperBodyCommandArray is unavailable in the selected AimDK overlay"
        )

    if len(frame.head_positions) != 2:
        raise HardwareContractError("upper-body head command requires 2 values")
    if len(frame.arm_positions) != 14:
        raise HardwareContractError("upper-body arm command requires 14 values")
    if frame.hand_sub_mode == 0:
        if frame.hand_positions:
            raise HardwareContractError(
                "hand_sub_mode=0 requires an empty hand command"
            )
    elif frame.hand_sub_mode == 1:
        if len(frame.hand_positions) != 2:
            raise HardwareContractError(
                "OmniPicker upper-body command requires [left, right]"
            )
        if not all(0.0 <= float(value) <= 1.0 for value in frame.hand_positions):
            raise HardwareContractError(
                "OmniPicker upper-body positions must be within [0, 1]"
            )
    else:
        raise HardwareContractError(
            "graspV2 only supports hand_sub_mode 0 or OmniPicker mode 1"
        )
    values = frame.head_positions + frame.arm_positions + frame.hand_positions
    if not all(abs(float(value)) < float("inf") for value in values):
        raise HardwareContractError("upper-body command contains non-finite values")

    message = types.UpperBodyCommandArray()
    message.header = types.MessageHeader()
    if stamp is not None:
        message.header.stamp = stamp
    message.header.frame_id = "mc_upper_body"
    message.header.sequence = int(sequence) % (2**32)
    message.source = config.upper_body.command_source
    message.hand_sub_mode = int(frame.hand_sub_mode)
    message.head_pos = [float(value) for value in frame.head_positions]
    message.arm_pos = [float(value) for value in frame.arm_positions]
    message.hand_pos = [float(value) for value in frame.hand_positions]
    return message


def _response_success(response: object) -> bool:
    common = getattr(response, "response", None)
    if common is None:
        return False
    status = getattr(getattr(common, "status", None), "value", None)
    code = getattr(getattr(common, "header", None), "code", None)
    if status is not None:
        return status == 1
    return code == 0


def _response_message(response: object) -> str:
    common = getattr(response, "response", None)
    if common is None:
        return "missing CommonResponse"
    return str(getattr(common, "message", "unknown service error"))


def _largest_tracking_error(
    feedback: Sequence[float],
    index_by_name: Mapping[str, int],
    targets: Mapping[str, float],
) -> tuple[float, str, float, float]:
    """Return error, joint name, feedback and target for the worst joint."""

    if not targets:
        raise HardwareContractError("tracking targets must not be empty")
    joint_name, target = max(
        targets.items(),
        key=lambda item: abs(
            feedback[index_by_name[item[0]]] - float(item[1])
        ),
    )
    actual = float(feedback[index_by_name[joint_name]])
    expected = float(target)
    return abs(actual - expected), joint_name, actual, expected


def _validate_return_endpoint(
    profile: RobotProfile,
    trajectory: JointTrajectory,
    *,
    tolerance_rad: float = 1e-6,
) -> None:
    """Require the independently planned return to end at the MC default pose."""

    default_by_name = dict(zip(profile.arm_pos_order, profile.mc_start_arm_pos()))
    error = max(
        abs(value - default_by_name[name])
        for name, value in zip(trajectory.joint_names, trajectory.positions[-1])
    )
    if error > tolerance_rad:
        raise LiveHardwareError(
            f"return trajectory ends {error:.6f} rad from the MC default arm pose"
        )


def _wait_for_control_services(
    required: Sequence[tuple[object, str]],
    timeout_s: float,
) -> None:
    """Give every required DDS service its own complete discovery window."""

    for client, name in required:
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise LiveHardwareError(
                "AimDK control service is unavailable after "
                f"{timeout_s:.1f} s: {name}"
            )


def create_aimdk_hardware_node(
    types: _RosTypes,
    config: HardwareConfig,
):
    """Create the concrete ``rclpy`` node class only after ROS is available."""

    require_aimdk_control_schema(types)
    rclpy = types.rclpy

    subscriber_qos = types.QoSProfile(
        reliability=types.ReliabilityPolicy.BEST_EFFORT,
        history=types.HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=types.DurabilityPolicy.VOLATILE,
    )
    publisher_qos = types.QoSProfile(
        reliability=types.ReliabilityPolicy.RELIABLE,
        history=types.HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=types.DurabilityPolicy.VOLATILE,
    )
    # Competition OmniPicker receiver contract from omnipicker_hand_student.py.
    hand_publisher_qos = types.QoSProfile(
        reliability=types.ReliabilityPolicy.BEST_EFFORT,
        history=types.HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=types.DurabilityPolicy.TRANSIENT_LOCAL,
    )

    class AimDKHardwareNode(types.Node):
        """Thin ROS adapter implementing the upper-body and hand ports."""

        def __init__(self):
            super().__init__("graspv2_aimdk_hardware")
            topics = config.topics
            self.config = config
            self.latest_states: dict[str, object | None] = {
                "arm": None,
                "waist": None,
                "head": None,
                "hand": None,
            }
            self.state_received_at: dict[str, float] = {}
            self.upper_body_sequence = 0
            self.upper_body_command_count = 0
            self.omnipicker_command_count = 0
            # Activation can publish HOLD frames before the requested path
            # starts.  Track planned motion separately so those setup frames
            # do not incorrectly suppress the safe animation fallback.
            self.planned_motion_started = False
            self.runtime_profile = os.environ.get(
                "GRASPV2_RUNTIME_PROFILE", "test"
            )
            self.omnipicker_sdk = (
                Path(__file__).resolve().parents[1]
                / "omnipicker_hand_student.py"
            )
            self.packed_temperature_notice_emitted = False
            self.split_mode_entered = False
            self.command_publishers = {
                "arm": self.create_publisher(
                    types.JointCommandArray, topics.arm_command, publisher_qos
                ),
                "waist": self.create_publisher(
                    types.JointCommandArray, topics.waist_command, publisher_qos
                ),
                "head": self.create_publisher(
                    types.JointCommandArray, topics.head_command, publisher_qos
                ),
                "hand": self.create_publisher(
                    types.HandCommandArray,
                    topics.hand_command,
                    hand_publisher_qos,
                ),
            }
            if types.UpperBodyCommandArray is not None:
                self.command_publishers["upper-body"] = self.create_publisher(
                    types.UpperBodyCommandArray,
                    topics.upper_body_command,
                    publisher_qos,
                )
            self.get_mode_client = self.create_client(
                types.GetMcAction, config.services.get_mc_action
            )
            self.set_mode_client = self.create_client(
                types.SetMcAction, config.services.set_mc_action
            )
            self.get_input_source_client = self.create_client(
                types.GetCurrentInputSource,
                config.services.get_current_input_source,
            )
            self.set_input_source_client = self.create_client(
                types.SetMcInputSource,
                config.services.set_mc_input_source,
            )
            for group, topic in (
                ("arm", topics.arm_state),
                ("waist", topics.waist_state),
                ("head", topics.head_state),
            ):
                self.create_subscription(
                    types.JointStateArray,
                    topic,
                    lambda message, selected=group: self._state_callback(
                        selected, message
                    ),
                    subscriber_qos,
                )
            self.create_subscription(
                types.HandStateArray,
                topics.hand_state,
                lambda message: self._state_callback("hand", message),
                subscriber_qos,
            )

        def _state_callback(self, group: str, message: object) -> None:
            self.latest_states[group] = message
            self.state_received_at[group] = time.monotonic()

        def _wait_for_state(self, group: str) -> object:
            deadline = time.monotonic() + config.upper_body.feedback_timeout_s
            while self.latest_states[group] is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            state = self.latest_states[group]
            if state is None:
                topic = getattr(config.topics, f"{group}_state")
                raise LiveHardwareError(f"no {group} feedback received from {topic}")
            return state

        def _wait_for_command_consumer(self, group: str) -> None:
            if group == "upper-body" and types.UpperBodyCommandArray is None:
                raise UpperBodyUnavailableError(
                    "UpperBodyCommandArray is unavailable in the selected "
                    "AimDK overlay"
                )
            publisher = self.command_publishers[group]
            deadline = time.monotonic() + config.upper_body.feedback_timeout_s
            while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if publisher.get_subscription_count() < 1:
                topic = (
                    config.topics.upper_body_command
                    if group == "upper-body"
                    else getattr(config.topics, f"{group}_command")
                )
                raise LiveHardwareError(
                    f"no AimDK command consumer discovered on {topic}"
                )

        def _wait_for_mode_services(self) -> None:
            # DDS endpoint discovery is independent of joint-state feedback.
            # Give every required service a complete discovery window: sharing
            # the 2 s feedback deadline made the last client fail intermittently
            # while a fresh participant was still discovering MC endpoints.
            timeout = config.upper_body.service_discovery_timeout_s
            _wait_for_control_services(
                (
                    (self.get_mode_client, config.services.get_mc_action),
                    (self.set_mode_client, config.services.set_mc_action),
                    (
                        self.get_input_source_client,
                        config.services.get_current_input_source,
                    ),
                    (
                        self.set_input_source_client,
                        config.services.set_mc_input_source,
                    ),
                ),
                timeout,
            )

        def _call_service(self, client: object, request: object, label: str):
            latest_error = ""
            for attempt in range(1, config.upper_body.service_retries + 1):
                try:
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(
                        self,
                        future,
                        timeout_sec=config.upper_body.service_timeout_s,
                    )
                    if future.done() and not future.cancelled():
                        error = future.exception()
                        if error is None and future.result() is not None:
                            return future.result()
                        latest_error = str(error or "empty response")
                    else:
                        future.cancel()
                except Exception as error:
                    latest_error = str(error)
                self.get_logger().warning(
                    f"{label} attempt {attempt}/"
                    f"{config.upper_body.service_retries} failed"
                )
            suffix = f": {latest_error}" if latest_error else ""
            raise LiveHardwareError(f"{label} failed after retries{suffix}")

        def get_mode(self) -> tuple[str, int]:
            request = types.GetMcAction.Request()
            request.request = types.CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()
            response = self._call_service(
                self.get_mode_client, request, "GetMcAction"
            )
            return str(response.info.action_desc), int(response.info.status.value)

        def set_mode(self, mode: str) -> None:
            request = types.SetMcAction.Request()
            request.header = types.RequestHeader()
            request.header.stamp = self.get_clock().now().to_msg()
            request.source = config.upper_body.mode_source
            request.command = types.McActionCommand()
            request.command.action_desc = mode
            response = self._call_service(
                self.set_mode_client, request, f"SetMcAction({mode})"
            )
            if not _response_success(response):
                raise LiveHardwareError(
                    f"SetMcAction({mode}) was rejected: "
                    f"{_response_message(response)}"
                )

        def get_input_source(self) -> tuple[str, int, int]:
            request = types.GetCurrentInputSource.Request()
            request.request = types.CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()
            response = self._call_service(
                self.get_input_source_client,
                request,
                "GetCurrentInputSource",
            )
            if not _response_success(response):
                raise LiveHardwareError(
                    "GetCurrentInputSource was rejected: "
                    f"{_response_message(response)}"
                )
            source = response.input_source
            return (
                str(source.name),
                int(source.priority),
                int(source.timeout),
            )

        def _set_input_source(self, action: int, label: str) -> bool:
            request = types.SetMcInputSource.Request()
            request.request = types.CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()
            request.action = types.McInputAction()
            request.action.value = int(action)
            request.input_source = types.McInputSource()
            request.input_source.name = config.upper_body.command_source
            request.input_source.priority = (
                config.upper_body.command_source_priority
            )
            request.input_source.timeout = (
                config.upper_body.command_source_timeout_ms
            )
            response = self._call_service(
                self.set_input_source_client,
                request,
                f"SetMcInputSource({label})",
            )
            return _response_success(response)

        def configure_input_source(self) -> None:
            """Register an independent source without overriding RC or VR."""

            # ADD is idempotent only at the policy level: an already registered
            # source is expected to reject ADD, after which MODIFY refreshes its
            # priority/timeout. Registration state resets when MC restarts.
            if not self._set_input_source(
                types.McInputAction.INPUTACTION_ADD,
                "ADD",
            ):
                if not self._set_input_source(
                    types.McInputAction.INPUTACTION_MODIFY,
                    "MODIFY",
                ):
                    raise LiveHardwareError(
                        "MC rejected both ADD and MODIFY for input source "
                        f"{config.upper_body.command_source!r}"
                    )
            if not self._set_input_source(
                types.McInputAction.INPUTACTION_ENABLE,
                "ENABLE",
            ):
                self.get_logger().warning(
                    "MC rejected ENABLE for input source "
                    f"{config.upper_body.command_source!r}; activation check "
                    "will determine whether it is already enabled"
                )
            self.get_logger().info(
                "MC input source configured: "
                f"name={config.upper_body.command_source!r}, "
                f"priority={config.upper_body.command_source_priority}, "
                f"timeout={config.upper_body.command_source_timeout_ms} ms"
            )

        def _wait_for_mode(self, expected: str) -> None:
            deadline = time.monotonic() + config.upper_body.mode_timeout_s
            latest = ("", -1)
            while time.monotonic() < deadline:
                latest = self.get_mode()
                if latest == (
                    expected,
                    config.upper_body.running_mode_status,
                ):
                    return
                rclpy.spin_once(self, timeout_sec=0.05)
            raise LiveHardwareError(
                f"MC mode did not become {expected}/"
                f"{config.upper_body.running_mode_status}; latest was "
                f"{latest[0]!r}/{latest[1]}"
            )

        def _require_stable_mode(self) -> None:
            mode, status = self.get_mode()
            expected = (
                config.upper_body.stable_mode,
                config.upper_body.running_mode_status,
            )
            if (mode, status) != expected:
                raise LiveHardwareError(
                    "upper-body split execution requires stable MC mode "
                    f"{expected[0]}/{expected[1]}; current mode is "
                    f"{mode!r}/{status}"
                )

        def _enter_split_mode(self) -> None:
            self.set_mode(config.upper_body.split_mode)
            # Restore stable mode even when status confirmation later times out.
            self.split_mode_entered = True
            self._wait_for_mode(config.upper_body.split_mode)

        def _activate_input_source(
            self,
            profile: RobotProfile,
            arm: Sequence[float],
            head: tuple[float, float],
        ) -> None:
            """Hold the live pose until MC selects graspV2's command source."""

            frame = UpperBodyFrame(
                head_positions=head,
                arm_positions=upper_body_arm_positions(profile, arm),
            )
            period = 1.0 / config.upper_body.command_rate_hz
            deadline = time.monotonic() + config.upper_body.feedback_timeout_s
            future = None
            request_started = 0.0
            latest = ("", -1, -1)
            while time.monotonic() < deadline:
                cycle = time.monotonic()
                self.command_upper_body(frame)
                rclpy.spin_once(self, timeout_sec=min(0.005, period))

                if future is None:
                    request = types.GetCurrentInputSource.Request()
                    request.request = types.CommonRequest()
                    request.request.header.stamp = (
                        self.get_clock().now().to_msg()
                    )
                    future = self.get_input_source_client.call_async(request)
                    request_started = time.monotonic()
                elif future.done() and not future.cancelled():
                    error = future.exception()
                    response = None if error is not None else future.result()
                    if response is not None and _response_success(response):
                        source = response.input_source
                        latest = (
                            str(source.name),
                            int(source.priority),
                            int(source.timeout),
                        )
                        if latest[0] == config.upper_body.command_source:
                            self.get_logger().info(
                                "MC selected input source "
                                f"{latest[0]!r} at priority {latest[1]}"
                            )
                            return
                    future = None
                elif (
                    time.monotonic() - request_started
                    > config.upper_body.service_timeout_s
                ):
                    future.cancel()
                    future = None

                remaining = period - (time.monotonic() - cycle)
                if remaining > 0.0:
                    time.sleep(remaining)

            if future is not None and not future.done():
                future.cancel()
            raise LiveHardwareError(
                "MC did not grant upper-body control to input source "
                f"{config.upper_body.command_source!r}; current source is "
                f"{latest[0]!r} at priority {latest[1]}"
            )

        def restore_stable_mode(self) -> None:
            if not self.split_mode_entered:
                return
            self.set_mode(config.upper_body.stable_mode)
            self._wait_for_mode(config.upper_body.stable_mode)
            self.split_mode_entered = False

        def _assert_fresh(self, group: str) -> None:
            age = time.monotonic() - self.state_received_at.get(group, 0.0)
            if age > config.upper_body.maximum_feedback_age_s:
                raise LiveHardwareError(
                    f"{group} feedback is stale ({age:.3f} s old)"
                )

        def _ordered_joints(
            self, state: object, expected_names: Sequence[str]
        ) -> tuple[object, ...]:
            joints = tuple(state.joints)
            # resolve_joint_feedback performs all name/count/order checks.
            resolve_joint_feedback(joints, expected_names)
            names = tuple(str(getattr(joint, "name", "")) for joint in joints)
            if all(not name for name in names):
                return joints
            by_name = {joint.name: joint for joint in joints}
            return tuple(by_name[name] for name in expected_names)

        def _assert_arm_health(
            self, profile: RobotProfile, *, require_stationary: bool
        ) -> tuple[float, ...]:
            state = self.latest_states["arm"]
            if state is None:
                state = self._wait_for_state("arm")
            self._assert_fresh("arm")
            domain = int(getattr(getattr(state, "state", None), "value", -1))
            if domain != 0:
                raise LiveHardwareError(f"arm domain is unhealthy: state={domain}")
            ordered = self._ordered_joints(state, profile.arm_pos_order)
            positions = resolve_joint_feedback(ordered, profile.arm_pos_order)
            velocities = tuple(float(joint.velocity) for joint in ordered)
            if not all(abs(value) < float("inf") for value in velocities):
                raise LiveHardwareError("arm feedback contains non-finite velocity")
            if require_stationary:
                fastest = max(abs(value) for value in velocities)
                limit = config.upper_body.maximum_start_velocity_rad_s
                if fastest > limit:
                    raise LiveHardwareError(
                        f"arm is moving at {fastest:.3f} rad/s; limit is {limit:.3f}"
                    )
            health = inspect_joint_health(ordered)
            if health.error_codes:
                name, code = health.error_codes[0]
                raise LiveHardwareError(f"joint {name} error_code={code}")
            if (
                health.hottest_temperature_c is not None
                and health.hottest_temperature_c
                >= config.upper_body.maximum_temperature_c
            ):
                raise LiveHardwareError(
                    "arm reached "
                    f"{health.hottest_temperature_c} C"
                )
            if (
                health.packed_legacy_temperatures
                and not self.packed_temperature_notice_emitted
            ):
                self.get_logger().warning(
                    "AimDK JointState error_code bytes match the legacy X2 "
                    "coil/motor temperature layout; treating the frame as "
                    f"temperature feedback (hottest="
                    f"{health.hottest_temperature_c} C)"
                )
                self.packed_temperature_notice_emitted = True
            return positions

        def _assert_head_health(self) -> tuple[float, float]:
            state = self.latest_states["head"]
            if state is None:
                state = self._wait_for_state("head")
            self._assert_fresh("head")
            domain = int(getattr(getattr(state, "state", None), "value", -1))
            if domain != 0:
                raise LiveHardwareError(f"head domain is unhealthy: state={domain}")
            joints = tuple(state.joints)
            if len(joints) != 2:
                raise LiveHardwareError(
                    f"head feedback requires 2 joints, received {len(joints)}"
                )
            positions = tuple(float(joint.position) for joint in joints)
            if not all(abs(value) < float("inf") for value in positions):
                raise LiveHardwareError("head feedback contains non-finite position")
            for joint in joints:
                if hasattr(joint, "error_code") and int(joint.error_code) != 0:
                    raise LiveHardwareError(
                        f"head joint {joint.name or '<unnamed>'} "
                        f"error_code={joint.error_code}"
                    )
            return positions

        def _assert_omnipicker_health(self, side: str | None = None) -> None:
            state = self.latest_states["hand"]
            if state is None:
                state = self._wait_for_state("hand")
            self._assert_fresh("hand")
            sides = (
                (side,)
                if side is not None
                else (config.omnipicker.installed_side,)
            )
            for selected in sides:
                hand_type = int(getattr(state, f"{selected}_hand_type").value)
                if hand_type != 2:
                    raise LiveHardwareError(
                        f"{selected} end effector is type {hand_type}, expected OmniPicker=2"
                    )
                hands = list(getattr(state, f"{selected}_hands"))
                if len(hands) != 1:
                    raise LiveHardwareError(
                        f"{selected} OmniPicker state requires one element, got {len(hands)}"
                    )
                if int(hands[0].faultcode) != 0:
                    raise LiveHardwareError(
                        f"{selected} OmniPicker faultcode={hands[0].faultcode}"
                    )
                if int(hands[0].state) == 3:
                    raise LiveHardwareError(
                        f"{selected} OmniPicker reports an object-drop state"
                    )

        def _check_omnipicker_feedback_if_available(
            self, side: str | None = None
        ) -> bool:
            if config.omnipicker.require_feedback:
                self._assert_omnipicker_health(side)
                return True
            state = self.latest_states["hand"]
            age = time.monotonic() - self.state_received_at.get("hand", 0.0)
            if state is None or age > config.upper_body.maximum_feedback_age_s:
                self.get_logger().warning(
                    "No fresh OmniPicker feedback; continuing with the "
                    "competition command-only contract"
                )
                return False
            self._assert_omnipicker_health(side)
            return True

        def preflight(
            self, profile: RobotProfile, component: str, transport: str
        ) -> None:
            if component in {"all", "upper-body"}:
                if transport == "upper-body":
                    require_aimdk_control_schema(
                        types, require_upper_body=True
                    )
                positions = self._assert_arm_health(
                    profile, require_stationary=True
                )
                if transport == "upper-body":
                    head = self._assert_head_health()
                    self._wait_for_mode_services()
                    mode, status = self.get_mode()
                    source = self.get_input_source()
                    self.get_logger().info(
                        f"MC upper-body interface ready: {len(positions)} arm "
                        f"joints, head={head}, current mode={mode!r}/{status}; "
                        f"input source={source[0]!r}/{source[1]}/"
                        f"{source[2]} ms; "
                        "preflight does not change mode"
                    )
                else:
                    self._wait_for_command_consumer("arm")
                    self.get_logger().info(
                        f"Upper-body arm HAL ready: {len(positions)} joints; "
                        f"waist/head ports configured at "
                        f"{config.topics.waist_command} and "
                        f"{config.topics.head_command}"
                    )
            if component in {"all", "omnipicker"}:
                self._wait_for_command_consumer("hand")
                feedback = self._check_omnipicker_feedback_if_available()
                self.get_logger().info(
                    "OmniPicker command interface is ready; "
                    f"feedback={'verified' if feedback else 'optional/unavailable'}"
                )

        def command_joint_group(
            self, group: str, commands: Sequence[JointSetpoint]
        ) -> None:
            if group not in {"arm", "waist", "head"}:
                raise HardwareContractError(
                    "upper-body joint group must be arm, waist, or head"
                )
            message = build_joint_message(types, commands)
            message.header.stamp = self.get_clock().now().to_msg()
            self.command_publishers[group].publish(message)

        def command_upper_body(self, frame: UpperBodyFrame) -> None:
            message = build_upper_body_message(
                types,
                config,
                frame,
                self.upper_body_sequence,
                self.get_clock().now().to_msg(),
            )
            self.upper_body_sequence = (self.upper_body_sequence + 1) % (2**32)
            self.command_publishers["upper-body"].publish(message)
            self.upper_body_command_count += 1

        def command_omnipicker(self, side: str, position: float) -> None:
            message = build_omnipicker_message(types, config, side, position)
            message.header.stamp = self.get_clock().now().to_msg()
            self.command_publishers["hand"].publish(message)
            self.omnipicker_command_count += 1

        def _competition_omnipicker_command(
            self,
            side: str,
            position: float,
            duration_s: float,
            *,
            hold_cycle: Callable[[], None] | None = None,
        ) -> None:
            """Run the repository OmniPicker SDK, optionally holding MC control."""

            require_installed_omnipicker_side(config, side)
            duration = float(duration_s)
            if duration <= 0.0:
                raise ValueError("OmniPicker command duration must be positive")
            if not self.omnipicker_sdk.is_file():
                raise LiveHardwareError(
                    "competition OmniPicker SDK is missing: "
                    f"{self.omnipicker_sdk}"
                )
            command = _competition_omnipicker_sdk_argv(
                self.omnipicker_sdk,
                side,
                position,
                duration,
            )
            self.omnipicker_command_count += 1
            try:
                process = subprocess.Popen(command, cwd=self.omnipicker_sdk.parent)
            except OSError as error:
                raise LiveHardwareError(
                    f"competition OmniPicker SDK could not start: {error}"
                ) from error
            deadline = time.monotonic() + duration + 5.0
            try:
                while process.poll() is None:
                    if time.monotonic() >= deadline:
                        process.terminate()
                        try:
                            process.wait(timeout=1.0)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=1.0)
                        raise LiveHardwareError(
                            "competition OmniPicker SDK timed out after "
                            f"{duration + 5.0:.1f} s"
                        )
                    if hold_cycle is None:
                        time.sleep(0.02)
                    else:
                        hold_cycle()
                return_code = process.wait()
            except Exception:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1.0)
                raise
            if return_code != 0:
                raise LiveHardwareError(
                    "competition OmniPicker SDK failed with status "
                    f"{return_code}"
                )
            self.get_logger().info(
                "Repository omnipicker_hand_student.py completed: "
                f"{side} target={position:.3f}, duration={duration:.3f} s"
            )

        def execute_omnipicker(self, side: str, position: float) -> None:
            if self.runtime_profile == "competition":
                self._check_omnipicker_feedback_if_available(side)
                self._competition_omnipicker_command(
                    side,
                    position,
                    config.omnipicker.publish_duration_s,
                )
                self._check_omnipicker_feedback_if_available(side)
                return
            self._wait_for_command_consumer("hand")
            self._check_omnipicker_feedback_if_available(side)
            period = 1.0 / config.omnipicker.publish_rate_hz
            deadline = time.monotonic() + config.omnipicker.publish_duration_s
            frames = 0
            while time.monotonic() < deadline:
                started = time.monotonic()
                self.command_omnipicker(side, position)
                frames += 1
                rclpy.spin_once(self, timeout_sec=min(0.005, period))
                remaining = period - (time.monotonic() - started)
                if remaining > 0.0:
                    time.sleep(remaining)
            if not self._check_omnipicker_feedback_if_available(side):
                self.get_logger().info(
                    f"{side} OmniPicker command completed without feedback; "
                    f"published {frames} frames"
                )
                return
            hand = list(
                getattr(self.latest_states["hand"], f"{side}_hands")
            )[0]
            error = abs(float(hand.position) - position)
            if (
                config.omnipicker.require_feedback
                and error > config.omnipicker.position_tolerance
            ):
                raise LiveHardwareError(
                    f"{side} OmniPicker final error {error:.3f} exceeds "
                    f"{config.omnipicker.position_tolerance:.3f}"
                )
            self.get_logger().info(
                f"{side} OmniPicker feedback={float(hand.position):.3f}, "
                f"target={position:.3f}; published {frames} frames"
            )

        def execute_upper_body_trajectory(
            self, profile: RobotProfile, trajectory: JointTrajectory
        ) -> None:
            """Execute through MC split mode while MC retains balance control."""

            require_aimdk_control_schema(types, require_upper_body=True)
            self._wait_for_mode_services()
            base = self._assert_arm_health(profile, require_stationary=True)
            head = self._assert_head_health()
            self._validate_upper_body_start(profile, trajectory, base)
            self._require_stable_mode()
            self.configure_input_source()
            try:
                self._enter_split_mode()
                self._wait_for_command_consumer("upper-body")
                self._activate_input_source(profile, base, head)
                self._run_upper_body_trajectory(
                    profile,
                    trajectory,
                    base,
                    head,
                    label="trajectory",
                )
            finally:
                self.restore_stable_mode()

        def _validate_upper_body_start(
            self,
            profile: RobotProfile,
            trajectory: JointTrajectory,
            current: Sequence[float],
        ) -> None:
            first = trajectory.sample(0.0)
            index_by_name = {
                name: index for index, name in enumerate(profile.arm_pos_order)
            }
            missing = sorted(set(first) - set(index_by_name))
            if missing:
                raise LiveHardwareError(
                    "trajectory does not match live profile: " + ", ".join(missing)
                )
            start_error = max(
                abs(current[index_by_name[name]] - position)
                for name, position in first.items()
            )
            if start_error > config.upper_body.maximum_start_error_rad:
                raise LiveHardwareError(
                    f"live arm is {start_error:.3f} rad from trajectory start; "
                    f"limit is {config.upper_body.maximum_start_error_rad:.3f}"
                )

        def _run_upper_body_trajectory(
            self,
            profile: RobotProfile,
            trajectory: JointTrajectory,
            base: Sequence[float],
            head: tuple[float, float],
            *,
            label: str,
        ) -> UpperBodyFrame:
            """Publish one segment while the caller owns the split-mode session."""

            current = self._assert_arm_health(profile, require_stationary=False)
            self._validate_upper_body_start(profile, trajectory, current)
            index_by_name = {
                name: index for index, name in enumerate(profile.arm_pos_order)
            }
            period = 1.0 / config.upper_body.command_rate_hz
            last_frame: UpperBodyFrame | None = None
            started = time.monotonic()
            tick = 0
            while True:
                target_time = min(tick * period, trajectory.duration)
                due = started + tick * period
                while time.monotonic() < due:
                    wait = min(0.005, due - time.monotonic())
                    rclpy.spin_once(self, timeout_sec=max(0.0, wait))
                lag = time.monotonic() - due
                if lag > max(0.06, period * 3.0):
                    raise LiveHardwareError(
                        "upper-body split command loop missed its deadline "
                        f"by {lag:.3f} s"
                    )
                positions = trajectory.sample(target_time)
                profile_positions = list(base)
                for name, value in positions.items():
                    profile_positions[index_by_name[name]] = value
                frame = UpperBodyFrame(
                    head_positions=head,
                    arm_positions=upper_body_arm_positions(
                        profile, profile_positions
                    ),
                )
                # Set this immediately before the first requested path frame,
                # after every validation that can still safely fall back.
                self.planned_motion_started = True
                self.command_upper_body(frame)
                last_frame = frame
                rclpy.spin_once(self, timeout_sec=0.0)
                feedback = self._assert_arm_health(
                    profile, require_stationary=False
                )
                self._assert_head_health()
                if target_time >= min(0.50, trajectory.duration):
                    (
                        tracking_error,
                        tracking_joint,
                        tracking_actual,
                        tracking_target,
                    ) = _largest_tracking_error(
                        feedback,
                        index_by_name,
                        positions,
                    )
                    if tracking_error > config.upper_body.maximum_tracking_error_rad:
                        raise LiveHardwareError(
                            f"arm tracking error {tracking_error:.3f} rad on "
                            f"{tracking_joint} at trajectory t={target_time:.3f} s "
                            f"(actual={tracking_actual:.3f}, "
                            f"target={tracking_target:.3f}) exceeds "
                            f"{config.upper_body.maximum_tracking_error_rad:.3f}"
                        )
                if target_time >= trajectory.duration:
                    break
                tick += 1

            assert last_frame is not None
            hold_deadline = time.monotonic() + config.upper_body.final_hold_s
            while time.monotonic() < hold_deadline:
                self._publish_hold_cycle(last_frame, period)
            self._assert_arm_health(profile, require_stationary=False)
            self.get_logger().info(
                f"MC upper-body {label} completed in {trajectory.duration:.3f} s"
            )
            return last_frame

        def _publish_hold_cycle(
            self,
            frame: UpperBodyFrame,
            period: float,
        ) -> None:
            cycle = time.monotonic()
            self.command_upper_body(frame)
            rclpy.spin_once(self, timeout_sec=min(0.005, period))
            remaining = period - (time.monotonic() - cycle)
            if remaining > 0.0:
                time.sleep(remaining)

        def _command_omnipicker_while_holding(
            self,
            profile: RobotProfile,
            frame: UpperBodyFrame,
            side: str,
            position: float,
            *,
            duration_s: float | None = None,
        ) -> None:
            self._check_omnipicker_feedback_if_available(side)
            period = 1.0 / config.upper_body.command_rate_hz
            duration = (
                config.omnipicker.publish_duration_s
                if duration_s is None
                else float(duration_s)
            )
            if not duration > 0.0:
                raise ValueError("OmniPicker command duration must be positive")
            if self.runtime_profile == "competition":

                def hold_cycle() -> None:
                    self._publish_hold_cycle(frame, period)
                    self._assert_arm_health(profile, require_stationary=False)

                self._competition_omnipicker_command(
                    side,
                    position,
                    duration,
                    hold_cycle=hold_cycle,
                )
                self._check_omnipicker_feedback_if_available(side)
                return
            deadline = time.monotonic() + duration
            frames = 0
            while time.monotonic() < deadline:
                cycle = time.monotonic()
                self.command_upper_body(frame)
                self.command_omnipicker(side, position)
                frames += 1
                rclpy.spin_once(self, timeout_sec=min(0.005, period))
                self._assert_arm_health(profile, require_stationary=False)
                remaining = period - (time.monotonic() - cycle)
                if remaining > 0.0:
                    time.sleep(remaining)
            self._check_omnipicker_feedback_if_available(side)
            self.get_logger().info(
                f"{side} OmniPicker target={position:.3f}; published {frames} "
                f"frames over {duration:.3f} s while holding upper-body pose"
            )

        def _hold_upper_body_pose(
            self,
            profile: RobotProfile,
            frame: UpperBodyFrame,
            duration_s: float,
            *,
            label: str,
        ) -> None:
            """Hold a planned endpoint for one deterministic state-machine phase."""

            duration = float(duration_s)
            if not duration > 0.0:
                raise ValueError("upper-body hold duration must be positive")
            period = 1.0 / config.upper_body.command_rate_hz
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self._publish_hold_cycle(frame, period)
                self._assert_arm_health(profile, require_stationary=False)
                self._assert_head_health()
            self.get_logger().info(
                f"MC upper-body {label} hold completed in {duration:.3f} s"
            )

        def _verify_while_holding(
            self,
            profile: RobotProfile,
            frame: UpperBodyFrame,
            verifier: Callable[[], VisualCheckResult],
            *,
            timeout_s: float,
            label: str,
        ) -> VisualCheckResult:
            """Keep sending the endpoint frame while a vision worker runs."""

            result: list[VisualCheckResult] = []
            errors: list[Exception] = []

            def run_verifier() -> None:
                try:
                    result.append(verifier())
                except Exception as error:  # propagated on the control thread
                    errors.append(error)

            worker = threading.Thread(
                target=run_verifier,
                name=f"graspv2_{label}_vision",
                daemon=True,
            )
            worker.start()
            deadline = time.monotonic() + timeout_s
            period = 1.0 / config.upper_body.command_rate_hz
            while worker.is_alive():
                if time.monotonic() >= deadline:
                    raise VisualVerificationError(
                        f"{label} vision verification exceeded {timeout_s:.1f} s"
                    )
                self._publish_hold_cycle(frame, period)
                self._assert_arm_health(profile, require_stationary=False)
                self._assert_head_health()
            worker.join(timeout=0.0)
            if errors:
                error = errors[0]
                if isinstance(error, VisualVerificationError):
                    raise error
                raise VisualVerificationError(
                    f"{label} vision verification failed: {error}"
                ) from error
            if len(result) != 1:
                raise VisualVerificationError(
                    f"{label} vision verification returned no result"
                )
            self.get_logger().info(f"{label} visual gate passed")
            return result[0]

        def execute_grasp_sequence(
            self,
            profile: RobotProfile,
            approach: JointTrajectory,
            lift: JointTrajectory,
            return_to_default: JointTrajectory,
            metadata: GraspPlanMetadata,
            verify_closed: Callable[[], VisualCheckResult],
            verify_lifted: Callable[[], VisualCheckResult],
            *,
            verification_timeout_s: float,
        ) -> tuple[VisualCheckResult, VisualCheckResult]:
            """Execute the same staged side-grasp state machine as MuJoCo."""

            # The MC fallback mirrors these physical arm/hand phases, but its
            # atomic playback cannot pause at the two conditional vision gates.
            require_aimdk_control_schema(types, require_upper_body=True)
            validate_trajectory_continuity(approach, lift)
            validate_trajectory_continuity(lift, return_to_default)
            require_installed_omnipicker_side(config, metadata.side)
            if metadata.robot_profile != profile.name:
                raise LiveHardwareError(
                    f"grasp plan is for {metadata.robot_profile}, not {profile.name}"
                )
            if verification_timeout_s <= 0.0:
                raise ValueError("verification_timeout_s must be positive")
            expected_joints = set(profile.right_arm_joints)
            if any(
                set(item.joint_names) != expected_joints
                for item in (approach, lift, return_to_default)
            ):
                raise LiveHardwareError(
                    "grasp trajectories do not command exactly the selected arm"
                )
            _validate_return_endpoint(profile, return_to_default)
            if not 0.0 < metadata.pregrasp_duration_s < approach.duration:
                raise LiveHardwareError(
                    "pregrasp/descent boundary is outside the approach trajectory"
                )

            move_above = slice_trajectory(
                approach, 0.0, metadata.pregrasp_duration_s
            )
            vertical_descent = slice_trajectory(
                approach, metadata.pregrasp_duration_s, approach.duration
            )
            controlled_lower = slice_trajectory(
                return_to_default,
                0.0,
                metadata.controlled_lower_duration_s,
            )
            open_hand_retreat = slice_trajectory(
                return_to_default,
                metadata.controlled_lower_duration_s,
                metadata.controlled_lower_duration_s
                + metadata.open_hand_retreat_duration_s,
            )
            closed_return = slice_trajectory(
                return_to_default,
                metadata.controlled_lower_duration_s
                + metadata.open_hand_retreat_duration_s,
                return_to_default.duration,
            )

            self._wait_for_mode_services()
            self._wait_for_command_consumer("hand")
            base = self._assert_arm_health(profile, require_stationary=True)
            head = self._assert_head_health()
            self._validate_upper_body_start(profile, approach, base)
            self._require_stable_mode()
            self.configure_input_source()

            reverse_approach = reverse_trajectory(approach)
            failed_open_retreat_duration = (
                approach.duration - metadata.pregrasp_duration_s
            )
            failed_open_retreat = slice_trajectory(
                reverse_approach,
                0.0,
                failed_open_retreat_duration,
            )
            failed_closed_return = slice_trajectory(
                reverse_approach,
                failed_open_retreat_duration,
                reverse_approach.duration,
            )
            closed_check: VisualCheckResult | None = None
            lifted_check: VisualCheckResult | None = None
            visual_failure: VisualVerificationError | None = None
            pregrasp_frame: UpperBodyFrame | None = None
            grasp_frame: UpperBodyFrame | None = None
            lift_frame: UpperBodyFrame | None = None
            try:
                # Prove that the legacy split action exists before starting
                # the planned arm path. Activation HOLD frames and the initial
                # empty-gripper command still leave animation fallback safe.
                if self.runtime_profile == "competition":
                    # Close the empty gripper before MC source activation so
                    # the standalone SDK cannot starve upper-body keepalives.
                    self.execute_omnipicker(
                        metadata.side,
                        config.omnipicker.closed_position,
                    )
                self._enter_split_mode()
                self._wait_for_command_consumer("upper-body")
                self._activate_input_source(profile, base, head)
                # MuJoCo starts with an empty closed gripper while moving above
                # the target. Establish the same state only after MC ownership
                # has been accepted.
                if self.runtime_profile != "competition":
                    self.execute_omnipicker(
                        metadata.side,
                        config.omnipicker.closed_position,
                    )
                base = self._assert_arm_health(
                    profile,
                    require_stationary=True,
                )
                pregrasp_frame = self._run_upper_body_trajectory(
                    profile,
                    move_above,
                    base,
                    head,
                    label="move-above-object",
                )
                self._command_omnipicker_while_holding(
                    profile,
                    pregrasp_frame,
                    metadata.side,
                    metadata.preopen_position,
                    duration_s=metadata.open_duration_s,
                )
                grasp_frame = self._run_upper_body_trajectory(
                    profile,
                    vertical_descent,
                    base,
                    head,
                    label="vertical-descent",
                )
                self._command_omnipicker_while_holding(
                    profile,
                    grasp_frame,
                    metadata.side,
                    metadata.grip_position,
                    duration_s=metadata.close_duration_s,
                )
                self._hold_upper_body_pose(
                    profile,
                    grasp_frame,
                    metadata.grasp_settle_duration_s,
                    label="grasp-settle",
                )
                try:
                    closed_check = self._verify_while_holding(
                        profile,
                        grasp_frame,
                        verify_closed,
                        timeout_s=verification_timeout_s,
                        label="closed-grasp",
                    )
                except VisualVerificationError as error:
                    visual_failure = error

                if visual_failure is None:
                    lift_frame = self._run_upper_body_trajectory(
                        profile,
                        lift,
                        base,
                        head,
                        label="two-second lift",
                    )
                    try:
                        lifted_check = self._verify_while_holding(
                            profile,
                            lift_frame,
                            verify_lifted,
                            timeout_s=verification_timeout_s,
                            label="post-lift",
                        )
                    except VisualVerificationError as error:
                        visual_failure = error

                # A successful lift is held for inspection, then reversed while
                # the object remains gripped.  Release occurs only after the
                # object has been placed back at the verified grasp position.
                if lift_frame is not None:
                    self._hold_upper_body_pose(
                        profile,
                        lift_frame,
                        metadata.lifted_hold_duration_s,
                        label="lifted-hold",
                    )
                    place_frame = self._run_upper_body_trajectory(
                        profile,
                        controlled_lower,
                        base,
                        head,
                        label="controlled-lower",
                    )
                    retreat = open_hand_retreat
                    final_return = closed_return
                else:
                    assert grasp_frame is not None
                    place_frame = grasp_frame
                    retreat = failed_open_retreat
                    final_return = failed_closed_return
                self._command_omnipicker_while_holding(
                    profile,
                    place_frame,
                    metadata.side,
                    config.omnipicker.open_position,
                    duration_s=metadata.release_duration_s,
                )
                self._hold_upper_body_pose(
                    profile,
                    place_frame,
                    metadata.place_settle_duration_s,
                    label="place-settle",
                )
                pregrasp_frame = self._run_upper_body_trajectory(
                    profile,
                    retreat,
                    base,
                    head,
                    label="open-hand-vertical-retreat",
                )
                self._command_omnipicker_while_holding(
                    profile,
                    pregrasp_frame,
                    metadata.side,
                    config.omnipicker.closed_position,
                    duration_s=metadata.reclose_duration_s,
                )
                self._run_upper_body_trajectory(
                    profile,
                    final_return,
                    base,
                    head,
                    label="return-to-default",
                )
                if visual_failure is not None:
                    raise visual_failure
            finally:
                self.restore_stable_mode()

            assert closed_check is not None and lifted_check is not None
            return closed_check, lifted_check

        def execute_arm_trajectory(
            self, profile: RobotProfile, trajectory: JointTrajectory
        ) -> None:
            self._wait_for_command_consumer("arm")
            base = self._assert_arm_health(profile, require_stationary=True)
            first = trajectory.sample(0.0)
            index_by_name = {
                name: index for index, name in enumerate(profile.arm_pos_order)
            }
            missing = sorted(set(first) - set(index_by_name))
            if missing:
                raise LiveHardwareError(
                    "trajectory does not match live profile: " + ", ".join(missing)
                )
            start_error = max(
                abs(base[index_by_name[name]] - position)
                for name, position in first.items()
            )
            if start_error > config.upper_body.maximum_start_error_rad:
                raise LiveHardwareError(
                    f"live arm is {start_error:.3f} rad from trajectory start; "
                    f"limit is {config.upper_body.maximum_start_error_rad:.3f}. "
                    "Move to the planned start pose before direct HAL execution."
                )

            period = 1.0 / config.upper_body.command_rate_hz
            started = time.monotonic()
            tick = 0
            last_commands: tuple[JointSetpoint, ...] | None = None
            while True:
                target_time = min(tick * period, trajectory.duration)
                due = started + tick * period
                while time.monotonic() < due:
                    wait = min(0.005, due - time.monotonic())
                    rclpy.spin_once(self, timeout_sec=max(0.0, wait))
                lag = time.monotonic() - due
                if lag > max(0.06, period * 3.0):
                    raise LiveHardwareError(
                        f"upper-body command loop missed its deadline by {lag:.3f} s"
                    )
                positions = trajectory.sample(target_time)
                velocities = trajectory_sample_velocity(
                    trajectory, target_time, period
                )
                commands = arm_setpoints(
                    profile, base, positions, velocities, config.upper_body
                )
                self.command_joint_group("arm", commands)
                last_commands = commands
                rclpy.spin_once(self, timeout_sec=0.0)
                feedback = self._assert_arm_health(
                    profile, require_stationary=False
                )
                if target_time >= min(0.50, trajectory.duration):
                    (
                        tracking_error,
                        tracking_joint,
                        tracking_actual,
                        tracking_target,
                    ) = _largest_tracking_error(
                        feedback,
                        index_by_name,
                        positions,
                    )
                    if tracking_error > config.upper_body.maximum_tracking_error_rad:
                        raise LiveHardwareError(
                            f"arm tracking error {tracking_error:.3f} rad on "
                            f"{tracking_joint} at trajectory t={target_time:.3f} s "
                            f"(actual={tracking_actual:.3f}, "
                            f"target={tracking_target:.3f}) exceeds "
                            f"{config.upper_body.maximum_tracking_error_rad:.3f}"
                        )
                if target_time >= trajectory.duration:
                    break
                tick += 1

            assert last_commands is not None
            hold_commands = tuple(
                JointSetpoint(
                    name=item.name,
                    position=item.position,
                    velocity=0.0,
                    effort=item.effort,
                    stiffness=item.stiffness,
                    damping=item.damping,
                )
                for item in last_commands
            )
            hold_deadline = time.monotonic() + config.upper_body.final_hold_s
            while time.monotonic() < hold_deadline:
                cycle = time.monotonic()
                self.command_joint_group("arm", hold_commands)
                rclpy.spin_once(self, timeout_sec=min(0.005, period))
                remaining = period - (time.monotonic() - cycle)
                if remaining > 0.0:
                    time.sleep(remaining)
            self._assert_arm_health(profile, require_stationary=False)
            self.get_logger().info(
                f"Direct upper-body trajectory completed in {trajectory.duration:.3f} s"
            )

    return AimDKHardwareNode()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "X2 AimDK upper-body/OmniPicker hardware bridge. Commands are "
            "dry-run unless --execute is explicitly supplied."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_HARDWARE_CONFIG_PATH,
        help="AimDK topic/tuning JSON",
    )
    subparsers = parser.add_subparsers(dest="operation", required=True)

    preflight = subparsers.add_parser(
        "preflight", help="read-only live interface and feedback checks"
    )
    preflight.add_argument("--robot", choices=tuple(PROFILES), default="ultra")
    preflight.add_argument(
        "--component",
        choices=("all", "upper-body", "omnipicker"),
        default="all",
    )
    preflight.add_argument(
        "--transport",
        choices=("upper-body", "hal-joint"),
        default="upper-body",
        help="upper-body transport checked by preflight",
    )

    trajectory = subparsers.add_parser(
        "trajectory", help="validate or directly publish a planned arm trajectory"
    )
    trajectory.add_argument("--robot", choices=tuple(PROFILES), default="ultra")
    trajectory.add_argument("--trajectory", type=Path, required=True)
    trajectory.add_argument(
        "--transport",
        choices=("upper-body", "hal-joint"),
        default="upper-body",
        help="MC split mode (default) or direct low-level HAL publication",
    )
    trajectory.add_argument(
        "--upper-body-fallback",
        choices=("animation", "none"),
        default="animation",
        help=(
            "when upper-body fails before planned trajectory motion starts, "
            "automatically "
            "run the same verified trajectory through MC animation (default: "
            "animation); use none to fail closed"
        ),
    )
    trajectory.add_argument(
        "--fallback-animation-output",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "output"
        / "upper_body_fallback.csv",
        help="generated hand-free MC CSV used only by the safe fallback",
    )
    trajectory.add_argument(
        "--fallback-speed-scale",
        type=float,
        default=0.5,
        help="fallback animation trajectory speed scale in (0, 1]",
    )
    trajectory.add_argument("--execute", action="store_true")
    trajectory.add_argument("--confirm-control-authority", action="store_true")

    hand = subparsers.add_parser(
        "omnipicker", help="validate or command one OmniPicker"
    )
    hand.add_argument(
        "--side",
        choices=AVAILABLE_GRIPPER_SIDES,
        default=INSTALLED_GRIPPER_SIDE,
        help="compatibility option; the installed OmniPicker is fixed on the right",
    )
    target = hand.add_mutually_exclusive_group(required=True)
    target.add_argument("--action", choices=("open", "close"))
    target.add_argument("--position", type=float)
    hand.add_argument("--execute", action="store_true")
    hand.add_argument("--confirm-control-authority", action="store_true")

    grasp = subparsers.add_parser(
        "grasp",
        help="execute a visually verified approach/close/two-second-lift sequence",
    )
    grasp.add_argument("--robot", choices=tuple(PROFILES), default="ultra")
    grasp.add_argument("--approach-trajectory", type=Path, required=True)
    grasp.add_argument("--lift-trajectory", type=Path, required=True)
    grasp.add_argument("--return-trajectory", type=Path, required=True)
    grasp.add_argument("--initial-vision", type=Path, required=True)
    grasp.add_argument("--target-class", required=True)
    grasp.add_argument(
        "--capture-backend",
        choices=("auto", "x2-aimdk", "orbbec-sdk"),
        default="auto",
    )
    grasp.add_argument("--camera-calibration", type=Path)
    grasp.add_argument("--vision-confidence", type=float, default=0.20)
    grasp.add_argument(
        "--image-rotation-deg",
        choices=("calibrated", "auto", "0", "180"),
        default="auto",
        help="RGB-D orientation mode forwarded to every visual verification capture",
    )
    grasp.add_argument(
        "--vision-runner",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "run_vision.sh",
    )
    grasp.add_argument(
        "--vision-output-root",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "output"
        / "grasp_verification",
    )
    grasp.add_argument(
        "--status",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "output"
        / "grasp_status.json",
    )
    grasp.add_argument("--verification-timeout", type=float, default=45.0)
    grasp.add_argument("--close-target-tolerance", type=float, default=0.08)
    grasp.add_argument("--lifted-target-tolerance", type=float, default=0.10)
    grasp.add_argument("--minimum-lift-ratio", type=float, default=0.60)
    grasp.add_argument("--maximum-lateral-drift", type=float, default=0.08)
    grasp.add_argument(
        "--upper-body-fallback",
        choices=("animation", "none"),
        default="animation",
        help=(
            "if upper-body grasp setup fails before planned trajectory motion "
            "starts, "
            "run the complete verified grasp/lift/place/return sequence through "
            "MC animation (default: animation); use none to fail closed"
        ),
    )
    grasp.add_argument(
        "--fallback-animation-output",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "output"
        / "grasp_fallback.csv",
        help="generated MC CSV used only by the pre-motion grasp fallback",
    )
    grasp.add_argument(
        "--fallback-speed-scale",
        type=float,
        default=1.0,
        help=(
            "grasp fallback motion speed scale in (0, 1] (default: 1.0, "
            "matching the verified simulation timing)"
        ),
    )
    grasp.add_argument("--execute", action="store_true")
    grasp.add_argument("--confirm-control-authority", action="store_true")
    return parser


def _resolve_position(args: argparse.Namespace, config: HardwareConfig) -> float:
    if args.position is not None:
        position = float(args.position)
    elif args.action == "open":
        position = config.omnipicker.open_position
    else:
        position = config.omnipicker.closed_position
    if not 0.0 <= position <= 1.0:
        raise HardwareContractError("--position must be within [0, 1]")
    return position


def _require_execution_confirmation(args: argparse.Namespace) -> None:
    if args.execute and not args.confirm_control_authority:
        raise HardwareContractError(
            "--execute requires --confirm-control-authority. Confirm the native "
            "MC ownership/mode policy, support/estop, and control of every joint "
            "needed to keep this robot safe before publishing commands."
        )
    if args.confirm_control_authority and not args.execute:
        raise HardwareContractError(
            "--confirm-control-authority is only valid with --execute"
        )


def _capture_visual_observation(
    args: argparse.Namespace,
    stage: str,
) -> VisualObservation:
    output_dir = args.vision_session_dir / stage
    command = [
        str(args.vision_runner),
        "--capture-backend",
        args.capture_backend,
        "--output-dir",
        str(output_dir),
        "--classes",
        args.target_class,
        "--target-class",
        args.target_class,
        "--conf",
        str(args.vision_confidence),
        "--image-rotation-deg",
        args.image_rotation_deg,
        "--device",
        "0",
    ]
    if args.camera_calibration is not None:
        command.extend(("--calibration", str(args.camera_calibration)))
    try:
        completed = subprocess.run(
            command,
            cwd=Path(__file__).resolve().parents[1],
            check=False,
            timeout=args.verification_timeout,
        )
    except subprocess.TimeoutExpired as error:
        raise VisualVerificationError(
            f"{stage} RGB-D capture/inference timed out"
        ) from error
    if completed.returncode != 0:
        raise VisualVerificationError(
            f"{stage} RGB-D capture/inference exited with {completed.returncode}"
        )
    return load_visual_observation(
        output_dir / "result.json",
        args.target_class,
    )


def _execute_visual_grasp(
    node: object,
    args: argparse.Namespace,
    profile: RobotProfile,
    approach: JointTrajectory,
    lift: JointTrajectory,
    return_to_default: JointTrajectory,
    metadata: GraspPlanMetadata,
) -> None:
    settings = VisualVerificationConfig(
        close_target_tolerance_m=args.close_target_tolerance,
        lifted_target_tolerance_m=args.lifted_target_tolerance,
        minimum_lift_ratio=args.minimum_lift_ratio,
        maximum_lateral_drift_m=args.maximum_lateral_drift,
    ).validate()
    checks: list[VisualCheckResult] = []
    closed_observation: list[VisualObservation] = []
    write_grasp_status(
        args.status,
        state="executing",
        metadata=metadata,
    )

    def verify_closed() -> VisualCheckResult:
        observation = _capture_visual_observation(args, "after_close")
        result = verify_closed_observation(
            observation,
            metadata.object_center_world_m,
            settings,
        )
        closed_observation.append(observation)
        checks.append(result)
        write_grasp_status(
            args.status,
            state="closed_grasp_verified",
            metadata=metadata,
            checks=checks,
        )
        return result

    def verify_lifted() -> VisualCheckResult:
        if len(closed_observation) != 1:
            raise VisualVerificationError(
                "post-lift verification has no closed-grasp observation"
            )
        observation = _capture_visual_observation(args, "after_lift")
        result = verify_lifted_observation(
            closed_observation[0],
            observation,
            metadata,
            settings,
        )
        checks.append(result)
        write_grasp_status(
            args.status,
            state="post_lift_verified",
            metadata=metadata,
            checks=checks,
        )
        return result

    try:
        node.execute_grasp_sequence(
            profile,
            approach,
            lift,
            return_to_default,
            metadata,
            verify_closed,
            verify_lifted,
            verification_timeout_s=args.verification_timeout,
        )
    except Exception as error:
        write_grasp_status(
            args.status,
            state="failed",
            metadata=metadata,
            checks=checks,
            error=str(error),
        )
        raise
    write_grasp_status(
        args.status,
        state="complete",
        metadata=metadata,
        checks=checks,
    )
    node.get_logger().info(
        "Visual grasp sequence passed; object followed the gripper through "
        f"the {metadata.lift_duration_s:.1f} s lift"
    )


def _animation_fallback_eligible(
    args: argparse.Namespace,
    node: object | None,
    fallback_animation: Path | None,
) -> bool:
    """Allow fallback until the requested arm trajectory actually starts."""

    competition_before_node = bool(
        os.environ.get("GRASPV2_RUNTIME_PROFILE") == "competition"
        and node is None
    )
    if node is None:
        before_planned_motion = competition_before_node
    else:
        planned_motion_started = getattr(node, "planned_motion_started", None)
        if planned_motion_started is None:
            # Preserve the conservative contract for older adapters and the
            # deliberately minimal test doubles that do not expose the marker.
            before_planned_motion = bool(
                getattr(node, "upper_body_command_count", 0) == 0
                and getattr(node, "omnipicker_command_count", 0) == 0
            )
        else:
            before_planned_motion = not bool(planned_motion_started)
    return bool(
        args.operation in {"trajectory", "grasp"}
        and (
            args.operation == "grasp"
            or getattr(args, "transport", None) == "upper-body"
        )
        and args.upper_body_fallback == "animation"
        and fallback_animation is not None
        and before_planned_motion
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        config = load_hardware_config(args.config)
        profile: RobotProfile | None = None
        trajectory: JointTrajectory | None = None
        lift_trajectory: JointTrajectory | None = None
        return_trajectory: JointTrajectory | None = None
        grasp_metadata: GraspPlanMetadata | None = None
        position: float | None = None
        fallback_animation: Path | None = None
        fallback_initial_gripper_position: float | None = None
        fallback_gripper_events: tuple[McGripperEvent, ...] = ()
        if args.operation in {"preflight", "trajectory", "grasp"}:
            profile = get_robot_profile(args.robot)
        if args.operation == "trajectory":
            _require_execution_confirmation(args)
            assert profile is not None
            trajectory = load_trajectory(
                args.trajectory,
                maximum_allowed_velocity=profile.maximum_velocity_rad_s + 1e-6,
            )
            if (
                args.transport == "upper-body"
                and args.upper_body_fallback == "animation"
            ):
                validate_animation_trajectory_source(args.trajectory, profile)
                animation = build_mc_animation(
                    trajectory,
                    speed_scale=args.fallback_speed_scale,
                    maximum_output_velocity=profile.maximum_velocity_rad_s,
                )
                fallback_animation = write_mc_animation_csv(
                    animation,
                    args.fallback_animation_output,
                )
                fallback_info = validate_mc_animation_csv(
                    fallback_animation,
                    maximum_velocity=profile.maximum_velocity_rad_s,
                )
            if not args.execute:
                topic = (
                    config.topics.upper_body_command
                    if args.transport == "upper-body"
                    else config.topics.arm_command
                )
                print(
                    f"Trajectory dry-run OK: {trajectory.frame_count} frames, "
                    f"{trajectory.duration:.3f} s, "
                    f"{trajectory.maximum_velocity:.3f} rad/s"
                )
                print(f"Transport: {args.transport}, topic={topic}")
                if fallback_animation is not None:
                    print(
                        "Safe fallback: MC animation prepared at "
                        f"{fallback_animation} "
                        f"({fallback_info.frame_count} frames, "
                        f"{fallback_info.duration_s:.3f} s); it is eligible "
                        "only before planned trajectory motion starts"
                    )
                print(
                    "Robot control: DISABLED (add both execution flags to publish)."
                )
                return 0
        elif args.operation == "omnipicker":
            _require_execution_confirmation(args)
            position = _resolve_position(args, config)
            if not args.execute:
                print(
                    f"OmniPicker dry-run OK: side={args.side}, position={position:.3f}, "
                    f"topic={config.topics.hand_command}"
                )
                print(
                    "Robot control: DISABLED (add both execution flags to publish)."
                )
                return 0
        elif args.operation == "grasp":
            _require_execution_confirmation(args)
            assert profile is not None
            trajectory = load_trajectory(
                args.approach_trajectory,
                maximum_allowed_velocity=profile.maximum_velocity_rad_s + 1e-6,
            )
            lift_trajectory = load_trajectory(
                args.lift_trajectory,
                maximum_allowed_velocity=profile.maximum_velocity_rad_s + 1e-6,
            )
            return_trajectory = load_trajectory(
                args.return_trajectory,
                maximum_allowed_velocity=profile.maximum_velocity_rad_s + 1e-6,
            )
            validate_trajectory_continuity(trajectory, lift_trajectory)
            validate_trajectory_continuity(lift_trajectory, return_trajectory)
            _validate_return_endpoint(profile, return_trajectory)
            grasp_metadata = load_grasp_plan_metadata(
                args.approach_trajectory,
                args.lift_trajectory,
                args.return_trajectory,
            )
            require_installed_omnipicker_side(config, grasp_metadata.side)
            if grasp_metadata.robot_profile != profile.name:
                raise HardwareContractError(
                    f"grasp plan is for {grasp_metadata.robot_profile}, "
                    f"not {profile.name}"
                )
            if abs(lift_trajectory.duration - grasp_metadata.lift_duration_s) > 1e-6:
                raise HardwareContractError(
                    "lift trajectory duration differs from its planning metadata"
                )
            initial_observation = load_visual_observation(
                args.initial_vision,
                args.target_class,
            )
            settings = VisualVerificationConfig(
                close_target_tolerance_m=args.close_target_tolerance,
                lifted_target_tolerance_m=args.lifted_target_tolerance,
                minimum_lift_ratio=args.minimum_lift_ratio,
                maximum_lateral_drift_m=args.maximum_lateral_drift,
            ).validate()
            verify_initial_observation(
                initial_observation,
                grasp_metadata.object_center_world_m,
                settings,
            )
            if args.upper_body_fallback == "animation":
                validate_animation_trajectory_source(
                    args.approach_trajectory,
                    profile,
                )
                animation = build_mc_grasp_animation(
                    trajectory,
                    lift_trajectory,
                    return_trajectory,
                    grasp_metadata,
                    speed_scale=args.fallback_speed_scale,
                    maximum_output_velocity=profile.maximum_velocity_rad_s,
                )
                fallback_initial_gripper_position = (
                    animation.initial_gripper_position
                )
                fallback_gripper_events = animation.gripper_events
                fallback_animation = write_mc_animation_csv(
                    animation,
                    args.fallback_animation_output,
                )
                fallback_info = validate_mc_animation_csv(
                    fallback_animation,
                    maximum_velocity=profile.maximum_velocity_rad_s,
                )
            if args.verification_timeout <= 0.0:
                raise HardwareContractError("--verification-timeout must be positive")
            if not 0.0 <= args.vision_confidence <= 1.0:
                raise HardwareContractError("--vision-confidence must be within [0, 1]")
            args.vision_runner = args.vision_runner.expanduser().resolve()
            if not args.vision_runner.is_file():
                raise HardwareContractError(
                    f"vision runner does not exist: {args.vision_runner}"
                )
            args.vision_session_dir = (
                args.vision_output_root.expanduser().resolve()
                / f"run_{time.time_ns()}"
            )
            if not args.execute:
                print(
                    "Visual grasp dry-run OK: "
                    f"robot={profile.name}, side={grasp_metadata.side}, "
                    f"approach={trajectory.duration:.3f} s, "
                    f"lift={lift_trajectory.duration:.3f} s/"
                    f"{grasp_metadata.lift_height_m:.3f} m, "
                    f"return={return_trajectory.duration:.3f} s, "
                    f"preopen={grasp_metadata.preopen_position:.3f}, "
                    f"grip={grasp_metadata.grip_position:.3f}"
                )
                print(
                    "Sequence: closed robot-side safe staging -> high transfer "
                    "-> fully open -> vertical descent -> radius close -> "
                    "visual gate -> lift -> visual no-drop gate -> hold -> "
                    "controlled lower -> placed release -> open retreat -> "
                    "close-empty -> verified return"
                )
                if fallback_animation is not None:
                    print(
                        "Safe pre-motion fallback: MC animation prepared at "
                        f"{fallback_animation} "
                        f"({fallback_info.frame_count} frames, "
                        f"{fallback_info.duration_s:.3f} s, "
                        f"{len(fallback_gripper_events)} synchronized hand "
                        "events). It is eligible only before planned arm "
                        "trajectory motion starts; setup HOLD frames and the "
                        "initial empty-gripper command remain recoverable. Its "
                        "physical phases match the "
                        "verified simulation sequence."
                    )
                print("Robot control/capture: DISABLED (add both execution flags).")
                return 0
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(str(error))

    if (
        os.environ.get("GRASPV2_RUNTIME_PROFILE") == "competition"
        and args.operation == "omnipicker"
    ):
        assert position is not None
        sdk = Path(__file__).resolve().parents[1] / "omnipicker_hand_student.py"
        if not sdk.is_file():
            print(
                f"Competition OmniPicker SDK is missing: {sdk}",
                file=sys.stderr,
            )
            return 1
        print(
            "Competition profile: executing the right OmniPicker through "
            f"{sdk.name}, target={position:.3f}"
        )
        return subprocess.run(
            _competition_omnipicker_sdk_argv(
                sdk,
                "right",
                position,
                config.omnipicker.publish_duration_s,
            ),
            cwd=sdk.parent,
            check=False,
        ).returncode

    node = None
    types = None
    fallback_reason: str | None = None
    try:
        types = _load_ros_types()
        configure_fastdds_logging()
        types.rclpy.init(args=[])
        node = create_aimdk_hardware_node(types, config)
        if args.operation == "preflight":
            assert profile is not None
            node.preflight(profile, args.component, args.transport)
            node.get_logger().info("Read-only preflight passed; no command published")
        elif args.operation == "trajectory":
            assert profile is not None and trajectory is not None
            if args.transport == "upper-body":
                node.execute_upper_body_trajectory(profile, trajectory)
            else:
                node.execute_arm_trajectory(profile, trajectory)
        elif args.operation == "omnipicker":
            assert position is not None
            node.execute_omnipicker(args.side, position)
        else:
            assert (
                profile is not None
                and trajectory is not None
                and lift_trajectory is not None
                and return_trajectory is not None
                and grasp_metadata is not None
            )
            _execute_visual_grasp(
                node,
                args,
                profile,
                trajectory,
                lift_trajectory,
                return_trajectory,
                grasp_metadata,
            )
    except Exception as error:
        # A competition overlay/service can fail with vendor-specific exception
        # types. Treat every ordinary runtime failure uniformly; the fallback
        # gate below still refuses replay once planned motion has begun.
        can_fallback = _animation_fallback_eligible(
            args,
            node,
            fallback_animation,
        )
        if can_fallback:
            fallback_reason = str(error)
            if node is not None:
                node.get_logger().warning(
                    "MC upper-body failed before planned trajectory motion "
                    "started; "
                    f"switching to the prepared animation fallback: {error}"
                )
            else:
                print(
                    "Competition local upper-body initialization failed before "
                    "creating any publisher; switching to the prepared MC "
                    f"animation fallback: {error}",
                    file=sys.stderr,
                )
        else:
            if node is not None:
                command_count = getattr(node, "upper_body_command_count", 0)
                hand_command_count = getattr(node, "omnipicker_command_count", 0)
                planned_motion_started = bool(
                    getattr(node, "planned_motion_started", False)
                )
                suffix = (
                    "; animation fallback blocked after planned motion started "
                    "with "
                    f"{command_count} upper-body and {hand_command_count} "
                    "OmniPicker command(s)"
                    if args.operation in {"trajectory", "grasp"}
                    and planned_motion_started
                    else ""
                )
                node.get_logger().error(
                    f"AimDK hardware operation aborted: {error}{suffix}"
                )
            else:
                print(
                    f"AimDK hardware operation aborted: {error}",
                    file=sys.stderr,
                )
            return 1
    finally:
        if node is not None:
            node.destroy_node()
        if types is not None and types.rclpy.ok():
            types.rclpy.shutdown()
    if fallback_reason is not None:
        assert fallback_animation is not None
        backend = Path(__file__).resolve().parents[1] / "tools" / "animation_backend.sh"
        if not backend.is_file():
            print(
                "Animation fallback could not start because the backend is "
                f"missing: {backend}",
                file=sys.stderr,
            )
            return 1
        if os.environ.get("GRASPV2_RUNTIME_PROFILE") == "competition":
            print(
                "The competition local upper-body interface failed before "
                "planned trajectory motion started; starting the complete local MC "
                "animation fallback: safe staging, open, descent, radius "
                "close, lift, hold, controlled lower, placed release, open "
                f"retreat, empty close and verified return ({fallback_reason}).",
                file=sys.stderr,
            )
        else:
            print(
                "Upper-body control aborted before planned trajectory motion "
                "started; "
                "starting the complete MC grasp animation fallback. The "
                "fallback performs safe staging, open, descent, radius close, "
                "lift, hold, controlled lower, placed release, open retreat, "
                f"empty close and verified return ({fallback_reason}).",
                file=sys.stderr,
            )
        backend_command = [
            str(backend),
            "--animation",
            str(fallback_animation),
            "--yes",
        ]
        if fallback_initial_gripper_position is not None:
            backend_command.extend(
                [
                    "--initial-gripper-position",
                    f"{fallback_initial_gripper_position:.9f}",
                ]
            )
        for event in fallback_gripper_events:
            backend_command.extend(
                [
                    "--gripper-event",
                    f"{event.time_s:.9f}:{event.position:.9f}:{event.label}",
                ]
            )
        return subprocess.run(
            backend_command,
            cwd=backend.parent.parent,
            check=False,
        ).returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
