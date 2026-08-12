"""Execute a grasp CSV through the compatible X2 MC animation player."""

from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path, PurePosixPath
import sys
import threading
import time
from typing import Callable, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from aimdk_msgs.msg import (
    CommonRequest,
    CommonState,
    DomainErrorState,
    JointState,
    JointStateArray,
    McAction,
    McActionStatus,
    McControlArea,
    McPresetMotion,
    RequestHeader,
)
from aimdk_msgs.srv import GetMcAction, SetMcPresetMotion

from .mc_animation import (
    DEFAULT_ANIMATION_NAME,
    DEFAULT_ARM_POSITION,
    McGripperEvent,
    validate_mc_animation_csv,
)
from .hardware_contract import inspect_joint_health
from .ros_logging import configure_fastdds_logging
from .trajectory import ARM_JOINT_ORDER


ARM_STATE_TOPIC = "/aima/hal/joint/arm/state"
GET_ACTION_SERVICE = "/aimdk_5Fmsgs/srv/GetMcAction"
PRESET_MOTION_SERVICE = "/aimdk_5Fmsgs/srv/SetMcPresetMotion"
DEFAULT_ROBOT_PATH = "/tmp/graspv2_mc_grasp_animation.csv"
DEFAULT_OMNIPICKER_STUDENT_SDK = (
    Path(__file__).resolve().parents[1] / "omnipicker_hand_student.py"
)

_CORE_ARM_JOINTS = tuple(
    name
    for name in ARM_JOINT_ORDER
    if "wrist_pitch" not in name and "wrist_roll" not in name
)


class SafetyError(RuntimeError):
    """Raised when live state is unsuitable for an MC-owned animation."""


def _load_omnipicker_student_sdk(path: Path):
    """Load the repository SDK without allowing its fallback re-exec."""

    source = path.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"repository OmniPicker SDK is missing: {source}")
    spec = importlib.util.spec_from_file_location(
        "_graspv2_omnipicker_hand_student",
        source,
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load repository OmniPicker SDK: {source}")
    module = importlib.util.module_from_spec(spec)
    flag = "_OMNIPICKER_STUDENT_REEXEC"
    previous_flag = os.environ.get(flag)
    os.environ[flag] = "1"
    try:
        spec.loader.exec_module(module)
    finally:
        if previous_flag is None:
            os.environ.pop(flag, None)
        else:
            os.environ[flag] = previous_flag
    return module


def _require_compatible_sdk_layout() -> None:
    joint_fields = set(JointState.get_fields_and_field_types())
    base_joint_fields = {"name", "position", "velocity", "effort"}
    allowed_joint_fields = (
        base_joint_fields | {"coil_temp", "motor_temp", "motor_vol"},
        base_joint_fields | {"error_code"},
    )
    if joint_fields not in allowed_joint_fields:
        raise RuntimeError(
            "wrong aimdk_msgs overlay: JointState does not match a supported "
            "temperature/error-code layout"
        )

    request_fields = set(
        SetMcPresetMotion.Request.get_fields_and_field_types()
    )
    base_request_fields = {"header", "area", "motion", "interrupt", "ani_path"}
    if request_fields not in (
        base_request_fields,
        base_request_fields | {"play_timestamp"},
    ):
        raise RuntimeError(
            "wrong aimdk_msgs overlay: SetMcPresetMotion_Request fields are "
            f"{sorted(request_fields)}; expected ani_path layout with optional "
            "play_timestamp"
        )


class McCustomGraspClient(Node):
    """Check live state, then submit one CSV path to MC's animation player."""

    def __init__(self, args: argparse.Namespace):
        super().__init__("x2_mc_custom_grasp_node")
        self.args = args
        self.arm_state: JointStateArray | None = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.arm_subscription = self.create_subscription(
            JointStateArray,
            ARM_STATE_TOPIC,
            self._arm_callback,
            qos,
        )
        self.get_action_client = self.create_client(
            GetMcAction,
            GET_ACTION_SERVICE,
        )
        self.preset_motion_client = self.create_client(
            SetMcPresetMotion,
            PRESET_MOTION_SERVICE,
        )
        self.omnipicker_sdk: object | None = None
        self.omnipicker_node: object | None = None
        self._gripper_close_attempted = False
        self._gripper_close_thread: threading.Thread | None = None
        self._gripper_close_succeeded = False
        self._gripper_event_index = 0
        self._gripper_event_failures: list[str] = []
        self._configure_gripper_best_effort()

    def _warn_gripper(self, message: str) -> None:
        self.get_logger().warning(
            f"OmniPicker best-effort warning: {message}; MC animation continues"
        )

    def _configure_gripper_best_effort(self) -> None:
        """Instantiate the repository student SDK without failing arm motion."""

        if self.args.no_gripper:
            return
        try:
            self.omnipicker_sdk = _load_omnipicker_student_sdk(
                self.args.omnipicker_sdk
            )
            self.omnipicker_node = self.omnipicker_sdk.OmniPickerStudentNode()
        except (Exception, SystemExit) as error:
            self.omnipicker_sdk = None
            self.omnipicker_node = None
            if getattr(self.args, "require_gripper_sdk", False):
                raise RuntimeError(
                    "competition OmniPicker SDK could not be loaded: "
                    f"{error}"
                ) from error
            self._warn_gripper(f"could not load repository student SDK: {error}")

    def _run_gripper_sdk_best_effort(
        self,
        action: str | float,
        *,
        label: str | None = None,
    ) -> bool:
        """Call the repository SDK's publish_command; never raise to MC."""

        if self.omnipicker_node is None:
            return False
        if isinstance(action, str):
            if action not in {"open", "close"}:
                raise ValueError(f"unsupported OmniPicker action: {action}")
            target_position = 1.0 if action == "open" else 0.0
            description = action
        else:
            target_position = float(action)
            if not 0.0 <= target_position <= 1.0:
                raise ValueError("OmniPicker position must be within [0, 1]")
            description = label or f"position={target_position:.3f}"
        try:
            self.omnipicker_node.publish_command("right", target_position)
            self.get_logger().info(
                "Repository omnipicker_hand_student SDK completed "
                f"{description} right"
            )
            return True
        except (Exception, SystemExit) as error:
            self._warn_gripper(
                f"student SDK {description} right failed: {error}"
            )
            return False

    def _run_close_worker(self) -> None:
        self._gripper_close_succeeded = self._run_gripper_sdk_best_effort(
            "close"
        )
        if not self._gripper_close_succeeded:
            self._gripper_event_failures.append("close")

    def _run_gripper_event_worker(self, event: McGripperEvent) -> None:
        succeeded = self._run_gripper_sdk_best_effort(
            event.position,
            label=event.label,
        )
        self._gripper_close_succeeded = (
            self._gripper_close_succeeded or succeeded
        )
        if not succeeded:
            self._gripper_event_failures.append(event.label)

    def _advance_gripper_events_best_effort(
        self,
        *,
        animation_elapsed_s: float,
    ) -> None:
        """Dispatch each due hand event without blocking arm observation."""

        events = tuple(getattr(self.args, "gripper_events", ()))
        if self._gripper_event_index >= len(events):
            return
        worker = self._gripper_close_thread
        if worker is not None and worker.is_alive():
            return
        event = events[self._gripper_event_index]
        if animation_elapsed_s + 1e-6 < event.time_s:
            return
        self._gripper_event_index += 1
        self._gripper_close_attempted = True
        if self.omnipicker_node is None:
            return
        delay = max(0.0, animation_elapsed_s - event.time_s)
        self.get_logger().info(
            f"Animation event {self._gripper_event_index}/{len(events)}: "
            f"{event.label}, right target={event.position:.3f}, "
            f"schedule_delay={delay:.3f} s"
        )
        self._gripper_close_thread = threading.Thread(
            target=self._run_gripper_event_worker,
            args=(event,),
            name=f"graspv2_omnipicker_event_{self._gripper_event_index}",
            daemon=True,
        )
        self._gripper_close_thread.start()

    def _advance_gripper_close_best_effort(
        self,
        *,
        animation_elapsed_s: float,
        now: float,
    ) -> None:
        """Start the repository SDK close call without blocking arm feedback."""

        del now
        if (
            self._gripper_close_attempted
            or animation_elapsed_s < self.args.gripper_close_time_s
        ):
            return
        self._gripper_close_attempted = True
        if self.omnipicker_node is None:
            return
        self.get_logger().info(
            "Animation reached the target hold; calling repository "
            "omnipicker_hand_student SDK: close right"
        )
        self._gripper_close_thread = threading.Thread(
            target=self._run_close_worker,
            name="graspv2_omnipicker_student_close",
            daemon=True,
        )
        self._gripper_close_thread.start()

    def shutdown_gripper_best_effort(self) -> None:
        """Reap the SDK worker and node without changing the command result."""

        thread = self._gripper_close_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
        if thread is not None and thread.is_alive():
            self._warn_gripper("student SDK close worker did not finish in time")
            return
        if self.omnipicker_node is not None:
            try:
                self.omnipicker_node.destroy_node()
            except Exception as error:
                self._warn_gripper(f"student SDK node cleanup failed: {error}")
            self.omnipicker_node = None

    def _arm_callback(self, message: JointStateArray) -> None:
        self.arm_state = message

    def wait_for_interfaces(self) -> None:
        for client, label in (
            (self.get_action_client, GET_ACTION_SERVICE),
            (self.preset_motion_client, PRESET_MOTION_SERVICE),
        ):
            if not client.wait_for_service(
                timeout_sec=self.args.interface_timeout
            ):
                raise RuntimeError(f"AimDK service is unavailable: {label}")

    def wait_for_safe_arm_state(self) -> None:
        deadline = time.monotonic() + self.args.interface_timeout
        while self.arm_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.arm_state is None:
            raise RuntimeError(f"no arm state received from {ARM_STATE_TOPIC}")

        if int(self.arm_state.state.value) != DomainErrorState.NONE:
            raise SafetyError(
                "arm domain is not healthy: "
                f"state={self.arm_state.state.value}"
            )
        joints = list(self.arm_state.joints)
        if len(joints) != 14:
            raise SafetyError(
                "expected an Ultra 14-joint arm state, received "
                f"{len(joints)} joints"
            )
        if (
            self.args.expected_arm_dof is not None
            and len(joints) != self.args.expected_arm_dof
        ):
            raise SafetyError(
                f"selected robot profile requires {self.args.expected_arm_dof} "
                f"arm joints, but live state contains {len(joints)}"
            )
        by_name = {joint.name: joint for joint in joints}
        if len(by_name) != len(joints):
            raise SafetyError("arm state contains duplicate joint names")
        missing = sorted(set(_CORE_ARM_JOINTS) - set(by_name))
        if missing:
            raise SafetyError(
                "arm state is missing core joints: " + ", ".join(missing)
            )

        health = inspect_joint_health(joints)
        if health.error_codes:
            raise SafetyError(
                "arm joints report non-zero error_code: "
                + ", ".join(
                    f"{name}={code}" for name, code in health.error_codes
                )
            )
        hottest = health.hottest_temperature_c
        if hottest is not None and hottest >= self.args.max_temperature:
            raise SafetyError(
                f"arm temperature {hottest} C reached the configured "
                f"{self.args.max_temperature} C threshold"
            )
        fastest = max(abs(float(joint.velocity)) for joint in joints)
        if fastest > self.args.max_start_velocity:
            raise SafetyError(
                f"arm is still moving at {fastest:.3f} rad/s; limit is "
                f"{self.args.max_start_velocity:.3f} rad/s"
            )

        default_by_name = dict(zip(ARM_JOINT_ORDER, DEFAULT_ARM_POSITION))
        position_error = max(
            abs(float(joint.position) - default_by_name[joint.name])
            for joint in joints
            if joint.name in default_by_name
        )
        if position_error > self.args.max_start_error:
            raise SafetyError(
                "arm is too far from the CSV start pose: maximum error is "
                f"{position_error:.3f} rad, limit is "
                f"{self.args.max_start_error:.3f} rad"
            )
        health_summary = (
            f"hottest={hottest} C"
            + (
                " (decoded from legacy packed temperature bytes)"
                if health.packed_legacy_temperatures
                else ""
            )
            if hottest is not None
            else "per-joint error_code=0"
        )
        self.get_logger().info(
            "Compatible AimDK arm state: "
            f"{len(joints)} joints, {health_summary}, "
            f"max_velocity={fastest:.3f} rad/s, "
            f"start_error={position_error:.3f} rad"
        )

    def _call_read_only(
        self,
        client: object,
        request_factory: Callable[[], object],
        label: str,
    ) -> object:
        last_error = ""
        for attempt in range(1, self.args.read_retries + 1):
            future = client.call_async(request_factory())
            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=self.args.service_timeout,
            )
            if future.done() and not future.cancelled():
                exception = future.exception()
                if exception is None and future.result() is not None:
                    return future.result()
                if exception is not None:
                    last_error = str(exception)
            else:
                future.cancel()
            self.get_logger().warning(
                f"{label} attempt {attempt}/{self.args.read_retries} failed"
            )
        suffix = f": {last_error}" if last_error else ""
        raise RuntimeError(f"{label} failed after retries{suffix}")

    def _call_mutating_once(
        self,
        client: object,
        request: object,
        label: str,
    ) -> object:
        # Mutating requests are deliberately not retried: a timed-out response
        # does not prove the MC failed to process the first request.
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.args.service_timeout,
        )
        if not future.done() or future.cancelled():
            future.cancel()
            raise RuntimeError(
                f"{label} response timed out; do not retry automatically"
            )
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"{label} failed: {exception}")
        response = future.result()
        if response is None:
            raise RuntimeError(f"{label} returned no response")
        return response

    def require_sd_mode(self) -> None:
        def request_factory() -> GetMcAction.Request:
            request = GetMcAction.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()
            return request

        response = self._call_read_only(
            self.get_action_client,
            request_factory,
            "GetMcAction",
        )
        code = int(response.header.code)
        action = int(response.info.current_action.value)
        status = int(response.info.status.value)
        description = str(response.info.action_desc)
        if code != 0:
            raise SafetyError(f"GetMcAction returned code={code}")
        # v0.9.0-rc13 sometimes leaves current_action.value at UNKNOWN (0)
        # while action_desc carries the authoritative enum name.
        standing = action == McAction.STAND_DEFAULT or (
            action == 0 and description == "STAND_DEFAULT"
        )
        if not standing or status != McActionStatus.RUNNING:
            raise SafetyError(
                "robot must already be stably standing; current action is "
                f"{description!r} ({action}), status={status}"
            )
        self.get_logger().info(
            "MC is in STAND_DEFAULT/100; no mode switch will be requested"
        )

    def _arm_positions(self) -> dict[str, float]:
        if self.arm_state is None:
            return {}
        return {
            joint.name: float(joint.position)
            for joint in self.arm_state.joints
            if joint.name in _CORE_ARM_JOINTS
        }

    def request_animation(self) -> int:
        """Submit once; a timeout is never retried because motion may start."""
        request = SetMcPresetMotion.Request()
        request.header = RequestHeader()
        request.header.stamp = self.get_clock().now().to_msg()
        request.area = McControlArea()
        request.area.value = self.args.area
        request.motion = McPresetMotion()
        request.motion.value = self.args.motion_id
        # Never interrupt another animation. If MC is busy, it must reject or
        # queue according to its own policy; this client does not force it.
        request.interrupt = False
        request.ani_path = self.args.robot_animation_path
        if hasattr(request, "play_timestamp"):
            request.play_timestamp = 0
        response = self._call_mutating_once(
            self.preset_motion_client,
            request,
            "SetMcPresetMotion(ani_path)",
        )
        common = response.response
        code = int(common.header.code)
        state = int(common.state.value)
        task_id = int(common.task_id)
        if code != 0 or state != CommonState.RUNNING:
            raise RuntimeError(
                "SetMcPresetMotion rejected: "
                f"code={code}, state={state}, task_id={task_id}"
            )
        self.get_logger().info(
            f"MC accepted animation task {task_id} through ani_path; "
            "the CSV now owns forward, hold and reverse-return playback"
        )
        return task_id

    def wait_for_motion_completion(
        self,
        initial_positions: dict[str, float],
    ) -> None:
        """Observe arm feedback until the CSV has moved and returned."""
        accepted_at = time.monotonic()
        start_deadline = accepted_at + self.args.motion_start_timeout
        completion_deadline = (
            accepted_at
            + self.args.animation_duration
            + self.args.completion_margin
        )
        started = False
        self._gripper_close_attempted = False
        self._gripper_close_succeeded = False
        self._gripper_event_index = 0
        self._gripper_event_failures = []
        while time.monotonic() < completion_deadline:
            self.arm_state = None
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.arm_state is None:
                continue
            positions = self._arm_positions()
            if not positions:
                continue
            fastest = max(
                abs(float(joint.velocity))
                for joint in self.arm_state.joints
                if joint.name in _CORE_ARM_JOINTS
            )
            initial_delta = max(
                abs(position - initial_positions[name])
                for name, position in positions.items()
                if name in initial_positions
            )
            if not started and (
                initial_delta >= self.args.motion_detection_threshold
                or fastest >= self.args.motion_detection_velocity
            ):
                started = True
                self.get_logger().info(
                    "Arm feedback confirms that MC animation playback started"
                )

            elapsed = time.monotonic() - accepted_at
            if started:
                events = tuple(getattr(self.args, "gripper_events", ()))
                if events:
                    self._advance_gripper_events_best_effort(
                        animation_elapsed_s=elapsed,
                    )
                else:
                    self._advance_gripper_close_best_effort(
                        animation_elapsed_s=elapsed,
                        now=time.monotonic(),
                    )
            if started and elapsed >= self.args.animation_duration:
                default_by_name = dict(
                    zip(ARM_JOINT_ORDER, DEFAULT_ARM_POSITION)
                )
                return_error = max(
                    abs(position - default_by_name[name])
                    for name, position in positions.items()
                )
                if (
                    return_error <= self.args.return_tolerance
                    and fastest <= self.args.return_velocity
                ):
                    self.get_logger().info(
                        "Reverse-return confirmed from arm feedback: "
                        f"max_error={return_error:.3f} rad, "
                        f"max_velocity={fastest:.3f} rad/s"
                    )
                    events = tuple(getattr(self.args, "gripper_events", ()))
                    if events and self._gripper_event_index != len(events):
                        raise RuntimeError(
                            "MC returned safely, but only "
                            f"{self._gripper_event_index}/{len(events)} staged "
                            "OmniPicker events were dispatched"
                        )
                    worker = self._gripper_close_thread
                    if worker is not None and worker.is_alive():
                        worker.join(timeout=3.0)
                    if (
                        worker is not None
                        and worker.is_alive()
                        and getattr(self.args, "require_gripper_sdk", False)
                    ):
                        raise RuntimeError(
                            "MC returned safely, but the competition OmniPicker "
                            "SDK worker did not finish"
                        )
                    if (
                        self._gripper_event_failures
                        and getattr(self.args, "require_gripper_sdk", False)
                    ):
                        raise RuntimeError(
                            "MC returned safely, but competition OmniPicker SDK "
                            "commands failed: "
                            + ", ".join(self._gripper_event_failures)
                        )
                    if events:
                        self.get_logger().info(
                            f"All {len(events)} staged OmniPicker events were "
                            "dispatched during MC playback"
                        )
                    elif self._gripper_close_succeeded:
                        self.get_logger().info(
                            "Repository OmniPicker SDK close completed without "
                            "blocking arm feedback"
                        )
                    return

            if not started and time.monotonic() >= start_deadline:
                raise RuntimeError(
                    "MC accepted the request, but no arm movement was "
                    "observed; check the robot MC log for animation_player "
                    "CSV/path/permission errors. The request was not retried."
                )

        if not started:
            raise RuntimeError(
                "MC accepted the request, but playback was not observed"
            )
        raise RuntimeError(
            "MC playback started, but return to the default arm pose was not "
            "confirmed before the observation timeout"
        )

    def preflight(self) -> None:
        """Run every read-only live check needed before submission."""
        self.wait_for_interfaces()
        self.require_sd_mode()
        self.wait_for_safe_arm_state()
        self.get_logger().info(
            "Read-only preflight passed; SetMcPresetMotion(ani_path) is "
            "available"
        )

    def execute(self) -> None:
        self.preflight()
        # Establish the planned initial hand state through the exact repository
        # student SDK before MC starts the atomic arm animation.
        # Competition mode requires this initial publication to succeed. During
        # playback MC still owns the safe return; any later SDK failure is
        # reported only after the arm has returned.
        initial_position = getattr(
            getattr(self, "args", None),
            "initial_gripper_position",
            None,
        )
        if initial_position is None:
            # Backward compatibility for callers constructed before staged
            # grasp animations existed.
            initial_succeeded = self._run_gripper_sdk_best_effort("open")
        else:
            initial_succeeded = self._run_gripper_sdk_best_effort(
                initial_position,
                label="initial-hand-state",
            )
        if (
            not initial_succeeded
            and getattr(
                getattr(self, "args", None),
                "require_gripper_sdk",
                False,
            )
        ):
            raise RuntimeError(
                "competition OmniPicker SDK initial command failed before "
                "animation playback"
            )
        # Re-check immediately before the only request that can start motion.
        self.require_sd_mode()
        self.arm_state = None
        self.wait_for_safe_arm_state()
        initial_positions = self._arm_positions()
        self.request_animation()
        self.wait_for_motion_completion(initial_positions)


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _gripper_event(value: str) -> McGripperEvent:
    """Parse TIME:POSITION:LABEL for the internal animation backend."""

    parts = value.split(":", 2)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "gripper event must use TIME:POSITION:LABEL"
        )
    try:
        time_s = float(parts[0])
        position = float(parts[1])
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "gripper event time and position must be numbers"
        ) from error
    label = parts[2].strip()
    if not time_s >= 0.0 or not time_s < float("inf"):
        raise argparse.ArgumentTypeError(
            "gripper event time must be finite and non-negative"
        )
    if not 0.0 <= position <= 1.0:
        raise argparse.ArgumentTypeError(
            "gripper event position must be within [0, 1]"
        )
    if not label:
        raise argparse.ArgumentTypeError("gripper event label must be non-empty")
    return McGripperEvent(time_s=time_s, position=position, label=label)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate and play an X2 v0.9 MC animation through "
            "SetMcPresetMotion(ani_path). MC remains running and no HAL arm "
            "command is published."
        )
    )
    parser.add_argument(
        "--animation",
        type=Path,
        default=_project_root() / DEFAULT_ANIMATION_NAME,
        help="local CSV copy used for validation",
    )
    parser.add_argument(
        "--robot-animation-path",
        default=DEFAULT_ROBOT_PATH,
        help="absolute CSV path visible to the robot's MC process",
    )
    parser.add_argument(
        "--omnipicker-sdk",
        type=Path,
        default=DEFAULT_OMNIPICKER_STUDENT_SDK,
        help="repository omnipicker_hand_student.py SDK path",
    )
    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="play only the arm animation without best-effort OmniPicker commands",
    )
    parser.add_argument(
        "--require-gripper-sdk",
        action="store_true",
        help=(
            "require repository omnipicker_hand_student.py loading and command "
            "success (competition profile)"
        ),
    )
    parser.add_argument(
        "--initial-gripper-position",
        type=float,
        default=1.0,
        help="right OmniPicker target established before animation playback",
    )
    parser.add_argument(
        "--gripper-event",
        dest="gripper_events",
        action="append",
        type=_gripper_event,
        default=[],
        metavar="TIME:POSITION:LABEL",
        help="repeatable hand target synchronized to the animation clock",
    )
    parser.add_argument(
        "--motion-id",
        type=int,
        default=9901,
        help=(
            "task marker carried in MotionState; the non-empty ani_path "
            "overrides MC's built-in motion lookup"
        ),
    )
    parser.add_argument(
        "--area",
        type=int,
        default=McControlArea.RIGHT_HAND,
        help="MC control-area marker (default: RIGHT_HAND=2)",
    )
    parser.add_argument("--max-velocity", type=float, default=0.5)
    parser.add_argument(
        "--expected-arm-dof",
        type=int,
        choices=(14,),
        default=14,
        help="require the live robot to expose the Ultra 14-joint arm state",
    )
    parser.add_argument("--max-temperature", type=int, default=80)
    parser.add_argument("--max-start-velocity", type=float, default=0.15)
    parser.add_argument("--max-start-error", type=float, default=0.40)
    parser.add_argument("--interface-timeout", type=float, default=5.0)
    parser.add_argument("--service-timeout", type=float, default=2.0)
    parser.add_argument("--read-retries", type=int, default=3)
    parser.add_argument("--motion-start-timeout", type=float, default=6.0)
    parser.add_argument("--completion-margin", type=float, default=15.0)
    parser.add_argument(
        "--motion-detection-threshold", type=float, default=0.02
    )
    parser.add_argument(
        "--motion-detection-velocity", type=float, default=0.02
    )
    parser.add_argument("--return-tolerance", type=float, default=0.10)
    parser.add_argument("--return-velocity", type=float, default=0.08)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--preflight",
        action="store_true",
        help="connect and run live read-only checks without starting motion",
    )
    mode.add_argument(
        "--execute",
        action="store_true",
        help="play ani_path through MC; otherwise only validate and print",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        local_animation = args.animation.expanduser().resolve()
        info = validate_mc_animation_csv(
            local_animation,
            maximum_velocity=args.max_velocity,
        )
    except (OSError, ValueError) as error:
        parser.error(str(error))
    args.animation_duration = info.duration_s
    args.gripper_close_time_s = info.grasp_close_time_s

    args.omnipicker_sdk = args.omnipicker_sdk.expanduser().resolve()

    robot_path = PurePosixPath(args.robot_animation_path)
    if not robot_path.is_absolute() or robot_path.suffix.lower() != ".csv":
        parser.error("--robot-animation-path must be an absolute .csv path")
    if args.max_temperature <= 0:
        parser.error("--max-temperature must be positive")
    if args.max_start_velocity <= 0.0:
        parser.error("--max-start-velocity must be positive")
    if args.max_start_error <= 0.0:
        parser.error("--max-start-error must be positive")
    if args.interface_timeout <= 0.0 or args.service_timeout <= 0.0:
        parser.error("timeouts must be positive")
    if args.read_retries < 1:
        parser.error("--read-retries must be at least one")
    positive_observation_values = (
        args.motion_start_timeout,
        args.completion_margin,
        args.motion_detection_threshold,
        args.motion_detection_velocity,
        args.return_tolerance,
        args.return_velocity,
    )
    if any(value <= 0.0 for value in positive_observation_values):
        parser.error("motion observation limits must be positive")
    if args.motion_id == 0:
        parser.error("--motion-id must be non-zero")
    if args.area < 0:
        parser.error("--area must be non-negative")
    if not 0.0 <= args.initial_gripper_position <= 1.0:
        parser.error("--initial-gripper-position must be within [0, 1]")
    if args.no_gripper and args.require_gripper_sdk:
        parser.error("--no-gripper conflicts with --require-gripper-sdk")
    if any(
        later.time_s <= earlier.time_s
        for earlier, later in zip(args.gripper_events, args.gripper_events[1:])
    ):
        parser.error("--gripper-event times must be strictly increasing")
    if any(event.time_s > info.duration_s for event in args.gripper_events):
        parser.error("--gripper-event time exceeds animation duration")

    print(f"Local MC animation: {local_animation}")
    print(f"Robot MC animation: {robot_path}")
    print(
        f"MC animation request: motion_id={args.motion_id}, area={args.area}, "
        "interrupt=false, ani_path=robot CSV"
    )
    print(
        f"Animation validation OK: {info.frame_count} frames, "
        f"{info.duration_s:.3f} s, max "
        f"{info.maximum_arm_velocity:.3f} rad/s"
    )
    if args.no_gripper:
        print("OmniPicker: DISABLED explicitly; arm animation only.")
    else:
        if args.gripper_events:
            print(
                "OmniPicker staged sequence: initial="
                f"{args.initial_gripper_position:.3f}; "
                + ", ".join(
                    f"{event.label}@{event.time_s:.3f}s="
                    f"{event.position:.3f}"
                    for event in args.gripper_events
                )
            )
        else:
            print(
                "OmniPicker: repository omnipicker_hand_student SDK opens "
                "before playback, then closes at "
                f"target hold t={info.grasp_close_time_s:.3f}s "
                f"(detected hold={info.grasp_hold_duration_s:.3f}s)."
            )
        print(f"OmniPicker SDK: {args.omnipicker_sdk}")
        if args.require_gripper_sdk:
            print(
                "OmniPicker policy: competition SDK is required; initial "
                "failure blocks motion and event failures are reported after "
                "MC completes its safe return."
            )
        else:
            print(
                "OmniPicker failure policy: warning only; SDK/DDS/controller/"
                "cable failures never abort arm playback or reverse-return."
            )
    print(
        "Return path: ENABLED inside the CSV; MC will play forward, hold, "
        "reverse the JSON, then return to its default arm pose."
    )
    print(
        "MC policy: KEEP RUNNING in STAND_DEFAULT; no SetMcAction call and "
        "no /aima/hal/joint/arm/command publisher."
    )
    print(
        "Playback API: SetMcPresetMotion(ani_path); RegisterCustomMotion and "
        "SetMcMotion are not used on this firmware."
    )
    if not args.execute and not args.preflight:
        print("Robot control: DISABLED (dry-run; add --execute to play).")
        return 0

    node: McCustomGraspClient | None = None
    try:
        _require_compatible_sdk_layout()
        configure_fastdds_logging()
        rclpy.init(args=[])
        node = McCustomGraspClient(args)
        if args.preflight:
            node.preflight()
            node.get_logger().info(
                "Robot control remains disabled: no SetMcPresetMotion request "
                "was sent"
            )
        else:
            node.execute()
    except (RuntimeError, SafetyError) as error:
        if node is not None:
            node.get_logger().error(f"Grasp request aborted: {error}")
        else:
            print(f"Grasp request aborted: {error}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.shutdown_gripper_best_effort()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
