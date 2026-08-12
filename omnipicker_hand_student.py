#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OmniPicker 双夹爪控制——参赛学生任务版

任务目标：
  参考智元灵犀 X2 AimDK Python 夹爪控制示例，补全本文件中的 TODO，
  使程序能够通过官方 ROS2 Hand HAL 接口控制左、右夹爪开合。

完成后应支持：
  python3 omnipicker_hand_student.py --publish open left
  python3 omnipicker_hand_student.py --publish close left
  python3 omnipicker_hand_student.py --publish open right
  python3 omnipicker_hand_student.py --publish close right
  python3 omnipicker_hand_student.py --publish position right 0.35

说明：
  1. 机器人模式和夹爪参数由赛项工作人员预先配置，不属于本任务内容。
  2. 本程序不得直接操作 CAN 或 EtherCAT。
  3. 初始文件不会发布有效夹爪控制命令，必须完成 TODO 后才能工作。
"""

import argparse
import glob
import os
import subprocess
import sys
import time


COMMAND_TOPIC = "/aima/hal/joint/hand/command"
LEFT_JOINT_NAME = "left_claw_joint"
RIGHT_JOINT_NAME = "right_claw_joint"
PUBLISH_FREQUENCY_HZ = 50.0
PUBLISH_DURATION_SECONDS = 2.0
_REEXEC_FLAG = "_OMNIPICKER_STUDENT_REEXEC"


def load_ros_environment():
    """尝试加载机器人上的 ROS 2 与 AimDK 环境。此函数无需修改。"""
    # graspV2 embeds this SDK after selecting the firmware-matched overlay.
    # Keep that environment intact instead of sourcing a second AimDK copy.
    if os.environ.get("GRASPV2_X2_ENV_READY") == "1":
        return
    setup_files = sorted(glob.glob("/opt/ros/*/setup.bash"))
    aimdk_setup = os.path.expanduser("~/aimdk/install/setup.bash")

    commands = []
    if setup_files:
        commands.append("source " + setup_files[0])
    if os.path.exists(aimdk_setup):
        commands.append("source " + aimdk_setup)

    if not commands:
        return

    result = subprocess.run(
        ["bash", "-c", " && ".join(commands) + " && env"],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError("ROS 2/AimDK 环境加载失败")

    for line in result.stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            os.environ[key] = value

    for path in os.environ.get("PYTHONPATH", "").split(":"):
        if path and path not in sys.path:
            sys.path.insert(0, path)


load_ros_environment()

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from aimdk_msgs.msg import HandCommand, HandCommandArray, HandType, MessageHeader
except ImportError as exc:
    if not os.environ.get(_REEXEC_FLAG):
        os.environ[_REEXEC_FLAG] = "1"
        os.execv(sys.executable, [sys.executable] + sys.argv)
    print("无法导入 ROS 2 或 AimDK Python 类型：", exc)
    print("请确认程序运行在已安装 AimDK 的 SoC1 开发计算单元上。")
    sys.exit(2)


def create_hand_command(joint_name, target_position):
    """创建单侧夹爪命令。

    TODO 1：
      根据官方示例创建 HandCommand，并填写名称、目标位置、速度、加速度、
      减速度和力参数，最后返回该消息对象。

    提示：target_position 已由参数解析部分限制在 0.0～1.0。
    """
    command = HandCommand()
    command.name = joint_name
    command.position = float(target_position)
    command.velocity = 1.0
    command.acceleration = 1.0
    command.deceleration = 1.0
    command.effort = 1.0
    return command


def build_hand_message(hand, target_position):
    """组装单侧夹爪的 HandCommandArray 消息。

    TODO 2：
      1. 创建 HandCommandArray 和消息头；
      2. 将目标侧设置为夹爪类型，并加入一条 HandCommand；
      3. 非目标侧应设置为无设备类型，且命令列表保持为空；
      4. 返回组装完成的消息。

    hand 的取值只会是 "left" 或 "right"。
    左右夹爪的逻辑关节名称已在文件顶部给出。
    """
    message = HandCommandArray()
    message.header = MessageHeader()
    message.header.frame_id = "hand_command"

    # NONE=0 表示本帧不控制该侧，CLAW=2 表示 OmniPicker 夹爪。
    message.left_hand_type = HandType(value=0)
    message.right_hand_type = HandType(value=0)
    message.left_hands = []
    message.right_hands = []

    if hand == "left":
        message.left_hand_type = HandType(value=2)
        message.left_hands = [
            create_hand_command(LEFT_JOINT_NAME, target_position)
        ]
    elif hand == "right":
        message.right_hand_type = HandType(value=2)
        message.right_hands = [
            create_hand_command(RIGHT_JOINT_NAME, target_position)
        ]
    else:
        raise ValueError(f"不支持的夹爪侧：{hand!r}")

    return message


class OmniPickerStudentNode(Node):
    """学生需要补全发布逻辑的 ROS 2 节点。"""

    def __init__(self):
        super().__init__("omnipicker_hand_student")

        # 当前赛事机器人夹爪链路使用以下 QoS。此部分无需修改。
        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            HandCommandArray,
            COMMAND_TOPIC,
            command_qos,
        )

    def publish_command(
        self,
        hand,
        target_position,
        duration_seconds=PUBLISH_DURATION_SECONDS,
    ):
        """在规定时间内持续发布目标夹爪命令。

        TODO 3：
          1. 调用 build_hand_message() 生成消息；
          2. 按 PUBLISH_FREQUENCY_HZ 持续发布；
          3. 发布时长使用 duration_seconds（默认
             PUBLISH_DURATION_SECONDS）；
          4. 循环期间保持 ROS 2 节点正常处理事件；
          5. 结束后输出实际发布帧数。

        注意：不要改为只发布一帧。
        """
        message = build_hand_message(hand, target_position)
        period_seconds = 1.0 / PUBLISH_FREQUENCY_HZ
        started_at = time.monotonic()
        duration_seconds = float(duration_seconds)
        if duration_seconds <= 0.0:
            raise ValueError("发布时长必须为正数")
        deadline = started_at + duration_seconds
        next_publish_at = started_at
        published_frames = 0

        while rclpy.ok() and time.monotonic() < deadline:
            stamp = self.get_clock().now().to_msg()
            message.header.stamp = stamp
            message.header.meas_stamp = stamp
            message.header.sequence = published_frames
            self.publisher.publish(message)
            published_frames += 1

            # 保持节点处理 ROS 2 事件，同时以单调时钟维持 50 Hz。
            rclpy.spin_once(self, timeout_sec=0.0)
            next_publish_at += period_seconds
            remaining = next_publish_at - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_publish_at = time.monotonic()

        print(
            f"已向 {hand} 夹爪发布 {published_frames} 帧，"
            f"目标位置 {target_position:.3f}。"
        )


def normalized_position(value):
    """解析并校验 0.0～1.0 的归一化夹爪开度。"""
    try:
        position = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("开度必须是数字") from exc

    if not 0.0 <= position <= 1.0:
        raise argparse.ArgumentTypeError("开度必须位于 0.0～1.0")
    return position


def parse_arguments():
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description="OmniPicker 双夹爪控制参赛学生任务版"
    )
    parser.add_argument(
        "--publish",
        action="store_true",
        help="完成 TODO 后，允许程序发布夹爪控制命令",
    )
    parser.add_argument(
        "action",
        choices=("open", "close", "position"),
        help="夹爪动作：open 打开，close 闭合，position 指定归一化开度",
    )
    parser.add_argument(
        "hand",
        choices=("left", "right"),
        help="目标夹爪：left 为左夹爪，right 为右夹爪",
    )
    parser.add_argument(
        "target_position",
        nargs="?",
        type=normalized_position,
        metavar="POSITION",
        help="position 动作的目标开度，范围 0.0～1.0",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=PUBLISH_DURATION_SECONDS,
        help="持续发布秒数（默认 2.0；GraspV2 用它匹配规划阶段时长）",
    )
    args = parser.parse_args()

    if args.action == "position" and args.target_position is None:
        parser.error("position 动作必须提供 0.0～1.0 的 POSITION")
    if args.action != "position" and args.target_position is not None:
        parser.error("只有 position 动作可以提供 POSITION")
    if args.duration <= 0.0:
        parser.error("--duration 必须为正数")
    return args


def main():
    args = parse_arguments()

    if not args.publish:
        print("未指定 --publish，程序不会发布控制命令。")
        print("完成全部 TODO 并确认现场安全后，再使用 --publish。")
        return

    # 本赛项定义：0.0 为闭合，1.0 为打开，中间值为部分开合。
    if args.action == "open":
        target_position = 1.0
    elif args.action == "close":
        target_position = 0.0
    else:
        target_position = args.target_position

    rclpy.init()
    node = OmniPickerStudentNode()
    try:
        node.publish_command(
            args.hand,
            target_position,
            duration_seconds=args.duration,
        )
    except NotImplementedError as exc:
        print("任务尚未完成：", exc)
        print("请根据配套说明和官方 AimDK 示例补全所有 TODO。")
        sys.exit(1)
    except KeyboardInterrupt:
        print("已停止控制程序。")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
