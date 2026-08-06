"""Execute a grasp CSV through the X2 v0.9 MC animation player."""

from __future__ import annotations

import argparse
from pathlib import Path, PurePosixPath
import sys
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
    validate_mc_animation_csv,
)
from .trajectory import ARM_JOINT_ORDER


ARM_STATE_TOPIC = "/aima/hal/joint/arm/state"
GET_ACTION_SERVICE = "/aimdk_5Fmsgs/srv/GetMcAction"
PRESET_MOTION_SERVICE = "/aimdk_5Fmsgs/srv/SetMcPresetMotion"
DEFAULT_ROBOT_PATH = "/tmp/graspv2_mc_grasp_animation.csv"

_CORE_ARM_JOINTS = tuple(
    name
    for name in ARM_JOINT_ORDER
    if "wrist_pitch" not in name and "wrist_roll" not in name
)


class SafetyError(RuntimeError):
    """Raised when live state is unsuitable for an MC-owned animation."""


def _require_sdk0907_layout() -> None:
    joint_fields = set(JointState.get_fields_and_field_types())
    expected_joint_fields = {
        "name",
        "position",
        "velocity",
        "effort",
        "coil_temp",
        "motor_temp",
        "motor_vol",
    }
    if joint_fields != expected_joint_fields:
        raise RuntimeError(
            "wrong aimdk_msgs overlay: JointState does not match the "
            "robot's v0.9.0.7 temperature layout"
        )

    expected_requests = (
        (
            SetMcPresetMotion.Request,
            {"header", "area", "motion", "interrupt", "ani_path"},
        ),
    )
    for request_type, expected_fields in expected_requests:
        actual = set(request_type.get_fields_and_field_types())
        if actual != expected_fields:
            raise RuntimeError(
                "wrong aimdk_msgs overlay: "
                f"{request_type.__name__} fields are {sorted(actual)}, "
                f"expected {sorted(expected_fields)}"
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
        if len(joints) not in (10, 14):
            raise SafetyError(
                "expected a 10/14-joint arm state, received "
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

        hottest = max(
            max(int(joint.coil_temp), int(joint.motor_temp))
            for joint in joints
        )
        if hottest >= self.args.max_temperature:
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
        self.get_logger().info(
            "Compatible v0.9 arm state: "
            f"{len(joints)} joints, hottest={hottest} C, "
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
        while time.monotonic() < completion_deadline:
            self.arm_state = None
            rclpy.spin_once(self, timeout_sec=0.05)
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
        # Re-check immediately before the only request that can start motion.
        self.require_sd_mode()
        self.arm_state = None
        self.wait_for_safe_arm_state()
        initial_positions = self._arm_positions()
        self.request_animation()
        self.wait_for_motion_completion(initial_positions)


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


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
        choices=(10, 14),
        help="require the live robot to match the selected youth/ultra profile",
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
    print("Hand control: DISABLED; the 20-column CSV has no hand fields.")
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
        _require_sdk0907_layout()
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
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
