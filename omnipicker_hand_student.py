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
    raise NotImplementedError("请完成 TODO 1：创建单侧夹爪命令")


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
    raise NotImplementedError("请完成 TODO 2：组装夹爪控制消息")


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

    def publish_command(self, hand, target_position):
        """在规定时间内持续发布目标夹爪命令。

        TODO 3：
          1. 调用 build_hand_message() 生成消息；
          2. 按 PUBLISH_FREQUENCY_HZ 持续发布；
          3. 发布时长使用 PUBLISH_DURATION_SECONDS；
          4. 循环期间保持 ROS 2 节点正常处理事件；
          5. 结束后输出实际发布帧数。

        注意：不要改为只发布一帧。
        """
        raise NotImplementedError("请完成 TODO 3：持续发布夹爪命令")


def parse_arguments():
    """解析命令行参数。此函数无需修改。"""
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
        choices=("open", "close"),
        help="夹爪动作：open 为打开，close 为闭合",
    )
    parser.add_argument(
        "hand",
        choices=("left", "right"),
        help="目标夹爪：left 为左夹爪，right 为右夹爪",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()

    if not args.publish:
        print("未指定 --publish，程序不会发布控制命令。")
        print("完成全部 TODO 并确认现场安全后，再使用 --publish。")
        return

    # 本赛项定义：0.0 为闭合，1.0 为打开。
    target_position = 1.0 if args.action == "open" else 0.0

    rclpy.init()
    node = OmniPickerStudentNode()
    try:
        node.publish_command(args.hand, target_position)
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
