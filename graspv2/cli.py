"""Single public command for simulation planning and MC animation replay."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

from .mc_animation import (
    build_mc_animation,
    validate_animation_trajectory_source,
    validate_mc_animation_csv,
    write_mc_animation_csv,
)
from .planner import (
    SimulatedGraspSequence,
    plan_lift_trajectory,
    plan_simulated_grasp_sequence,
    plan_trajectory,
    preview_scene,
    replay_grasp_sequence,
    replay_viewer,
)
from .robot_profiles import (
    AVAILABLE_GRIPPER_SIDES,
    INSTALLED_GRIPPER_SIDE,
    PROFILES,
    RobotProfile,
    get_robot_profile,
)
from .trajectory import TrajectoryValidationError, load_trajectory


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY = ROOT / "output" / "planned_trajectory.json"
DEFAULT_REPORT = ROOT / "output" / "planning_report.json"
DEFAULT_ANIMATION = ROOT / "output" / "mc_animation.csv"
DEFAULT_VISION_RESULT = ROOT / "output" / "result.json"
DEFAULT_DEMO_SCENE = ROOT / "config" / "mujoco_demo_scene.json"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="./run.sh",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "X2 路径规划、MuJoCo 仿真验证与 MC animation 播放。\n"
            "默认读取 output/result.json，生成视觉桌子并打开 MuJoCo Viewer，\n"
            "使用 --headless 可关闭显示；始终不连接机器人，也不发送控制指令。"
        ),
        epilog="""示例：
  ./run.sh
      使用上次视觉夹爪中心生成桌子，规划后打开 MuJoCo Viewer。

  ./run.sh --headless
      使用同一桌子和规划，但不打开窗口，适用于 SSH/Orin。

  ./run.sh --vision-result output/result.json
      显式指定另一份视觉结果；语义与默认路径相同。

  ./run.sh --target 0.38 -0.30 0.92
      保留默认视觉桌面，仅用手工世界坐标覆盖视觉夹爪中心。

  ./run.sh --no-vision
      不读取视觉 JSON，不生成桌面障碍，改用内置可达测试目标。

  ./run.sh --demo-scene
      使用内置 X2 Ultra、右侧 OmniPicker、桌子和杯子场景并打开 Viewer。

  ./run.sh --mode animation
      把已验证轨迹转换为 CSV；完全离线，不连接机器人。

  ./run.sh --mode animation --animation /path/to/action.csv
      校验并准备重放已有 CSV，不重新规划或覆盖该文件。

  ./run.sh --mode animation --execute
      真机只读预检、上传校验并在再次输入 RUN 后播放。

坐标约定：米；+X 向机器人前方，+Y 向机器人左侧，+Z 向上。
退出码：0 成功；1 规划/校验/执行失败；2 命令行参数错误。""",
    )
    parser.add_argument(
        "--robot",
        choices=tuple(PROFILES),
        default="ultra",
        help="兼容旧命令的机型参数；当前只支持 ultra（默认）",
    )
    parser.add_argument(
        "--mode",
        choices=("sim", "animation"),
        default="sim",
        help="运行模式（默认 sim）；sim=规划仿真，animation=转换或重放 CSV",
    )
    parser.add_argument(
        "--side",
        choices=AVAILABLE_GRIPPER_SIDES,
        default=INSTALLED_GRIPPER_SIDE,
        help="抓取手臂（固定 right）；比赛机器人仅右手安装 OmniPicker",
    )
    parser.add_argument(
        "--robot-urdf",
        type=Path,
        help=(
            "实际 X2 全身 visual URDF；默认依次检查 GRASPV2_ROBOT_URDF、"
            "~/x2_arm_sim 和 ~/下载/Downloads 中的 X2_URDF-v1.3.0"
        ),
    )
    parser.add_argument(
        "--target",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help=(
            "实际抓取 TCP 的目标世界坐标，单位米；默认取视觉夹爪中心；"
            "与视觉结果同用时只覆盖夹爪中心，仍保留视觉桌面障碍"
        ),
    )
    vision_group = parser.add_mutually_exclusive_group()
    vision_group.add_argument(
        "--vision-result",
        type=Path,
        help=(
            f"视觉结果 JSON（默认 {DEFAULT_VISION_RESULT}），读取 "
            "object_center_mujoco_m、gripper_center_mujoco_m 和桌面碰撞盒"
        ),
    )
    vision_group.add_argument(
        "--no-vision",
        action="store_true",
        help="禁用默认视觉结果和桌面障碍；无 --target 时使用内置可达测试目标",
    )
    vision_group.add_argument(
        "--demo-scene",
        action="store_true",
        help="使用内置 X2 Ultra + 右侧 OmniPicker + 桌子/杯子 MuJoCo 场景",
    )
    parser.add_argument(
        "--trajectory",
        type=Path,
        default=DEFAULT_TRAJECTORY,
        help=f"轨迹 JSON 输出/输入路径（默认 {DEFAULT_TRAJECTORY}）",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=DEFAULT_REPORT,
        help=f"规划验证报告输出路径（默认 {DEFAULT_REPORT}）",
    )
    parser.add_argument(
        "--lift-trajectory",
        type=Path,
        help="同时生成从夹爪中心目标向桌面法向抬升的独立碰撞验证轨迹",
    )
    parser.add_argument(
        "--lift-report",
        type=Path,
        help="抬升轨迹验证报告；默认使用 lift trajectory 同目录下的 report 文件",
    )
    parser.add_argument(
        "--return-trajectory",
        type=Path,
        help="同时生成从抬升点安全返回默认位的独立碰撞验证轨迹",
    )
    parser.add_argument(
        "--return-report",
        type=Path,
        help="返回轨迹验证报告；默认使用 return trajectory 同目录下的 report 文件",
    )
    parser.add_argument(
        "--lift-height",
        type=float,
        default=0.045,
        help="视觉验证抓取使用的抬升距离，单位米（默认 0.045）",
    )
    parser.add_argument(
        "--lift-duration",
        type=float,
        default=2.5,
        help="抬升动作时长，单位秒（默认 2.5）",
    )
    parser.add_argument(
        "--animation",
        type=Path,
        help=(
            "已有 MC animation CSV；仅用于 animation 模式，提供后跳过规划和转换，"
            "只校验/重放且不修改原文件"
        ),
    )
    parser.add_argument(
        "--table-clearance",
        type=float,
        default=0.025,
        help="机器人代理几何到视觉桌面的最小距离，单位米（默认 0.025，范围 0~0.10）",
    )
    parser.add_argument(
        "--approach-distance",
        type=float,
        default=0.03,
        help=(
            "识别到物体时，预抓点高于物体估计顶部的距离；无物体模型时为"
            "预抓点到目标的距离，单位米（默认 0.03，范围 0.01~0.30）"
        ),
    )
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=0.5,
        help="生成 animation 时的轨迹速度倍率（默认 0.5，范围 0<值<=1）",
    )
    display_group = parser.add_mutually_exclusive_group()
    display_group.add_argument(
        "--headless",
        action="store_true",
        help="sim 模式只做规划和验证，不打开 MuJoCo Viewer",
    )
    display_group.add_argument(
        "--viewer",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "animation 模式连接真机：先只读预检，再上传校验并请求播放；"
            "不提供时始终离线"
        ),
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="与 --execute 同用，跳过最终输入 RUN；仅用于已建立额外安全措施的自动化",
    )
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.execute and args.mode != "animation":
        parser.error("--execute is only valid with --mode animation")
    if args.yes and not args.execute:
        parser.error("--yes requires --mode animation --execute")
    if args.viewer and args.mode != "sim":
        parser.error("--viewer is only valid with --mode sim")
    if args.headless and args.mode != "sim":
        parser.error("--headless is only valid with --mode sim")
    if args.animation is not None and args.mode != "animation":
        parser.error("--animation is only valid with --mode animation")
    if args.animation is not None and (
        args.target is not None
        or args.vision_result is not None
        or args.no_vision
        or args.demo_scene
    ):
        parser.error("an existing --animation cannot be combined with planning inputs")
    if args.lift_report is not None and args.lift_trajectory is None:
        parser.error("--lift-report requires --lift-trajectory")
    if args.return_report is not None and args.return_trajectory is None:
        parser.error("--return-report requires --return-trajectory")
    if args.return_trajectory is not None and args.lift_trajectory is None:
        parser.error("--return-trajectory requires --lift-trajectory")
    if args.lift_trajectory is not None and args.mode != "sim":
        parser.error("--lift-trajectory is only valid with --mode sim")
    if args.return_trajectory is not None and args.mode != "sim":
        parser.error("--return-trajectory is only valid with --mode sim")


def _resolve_vision_result(args: argparse.Namespace) -> Path | None:
    if args.no_vision:
        return None
    if args.demo_scene:
        if not DEFAULT_DEMO_SCENE.is_file():
            raise RuntimeError(
                f"built-in MuJoCo demo scene is missing: {DEFAULT_DEMO_SCENE}"
            )
        return DEFAULT_DEMO_SCENE.resolve()
    source = (
        args.vision_result.expanduser().resolve()
        if args.vision_result is not None
        else DEFAULT_VISION_RESULT
    )
    if not source.is_file():
        if args.vision_result is None:
            raise RuntimeError(
                f"default vision result does not exist: {source}; run vision first, "
                "pass --vision-result PATH, or use --no-vision for the built-in test"
            )
        raise RuntimeError(f"vision result does not exist: {source}")
    return source


def _grasp_side(args: argparse.Namespace) -> str:
    """Return the only arm fitted with the competition OmniPicker."""
    if args.side != INSTALLED_GRIPPER_SIDE:
        raise RuntimeError(
            "grasp planning is restricted to the right arm because the "
            "competition robot has no left OmniPicker"
        )
    return INSTALLED_GRIPPER_SIDE


def _plan_with_installed_gripper(
    args: argparse.Namespace,
    profile: RobotProfile,
    vision_result: Path | None,
):
    return plan_trajectory(
        profile,
        side=_grasp_side(args),
        target_world_xyz=args.target,
        vision_result=vision_result,
        robot_urdf=args.robot_urdf,
        table_clearance_m=args.table_clearance,
        approach_distance_m=args.approach_distance,
    )


def _plan(args: argparse.Namespace, profile: RobotProfile):
    vision_result = _resolve_vision_result(args)
    if args.demo_scene:
        print(f"MuJoCo demo scene: {vision_result}")
    elif vision_result is not None:
        print(f"Vision obstacle: {vision_result}")
    else:
        print("Vision obstacle: disabled (--no-vision)")
    result = _plan_with_installed_gripper(args, profile, vision_result)
    trajectory = args.trajectory.expanduser().resolve()
    report = args.report.expanduser().resolve()
    print(f"Robot: {profile.name} ({profile.arm_dof} DoF/arm)")
    print("IK: official x2_ik_sdk")
    print(f"Kinematic URDF: {result.report['kinematic_urdf']}")
    visual_urdf = result.report["robot_visual_urdf"]
    print(
        f"Robot visual URDF: {visual_urdf}"
        if visual_urdf is not None
        else "Robot appearance: full-body proxies generated from kinematic URDF"
    )
    omnipicker_description = result.report["omnipicker_visual_description"]
    print(
        f"OmniPicker visual: {omnipicker_description}"
        if omnipicker_description is not None
        else "OmniPicker visual: not found (collision boxes only)"
    )
    print(f"Trajectory: {trajectory}")
    print(f"Report: {report}")
    print(
        "Simulation gate: PASS; "
        f"frames={len(result.times)}, duration={result.duration_s:.3f}s, "
        f"final_error={result.report['final_position_error_m']:.6f}m"
    )
    if result.report.get("nearest_ik_fallback_used"):
        print(
            "IK nearest-point fallback: ACCEPTED after collision/edge checks; "
            f"waypoints={result.report['nearest_ik_fallback_count']}, "
            "maximum_position_error="
            f"{result.report['nearest_ik_fallback_maximum_error_m']:.6f}m, "
            f"limit={result.report['nearest_ik_fallback_limit_m']:.3f}m"
        )
    sequence: SimulatedGraspSequence | None = None
    if result.obstacle is not None and result.obstacle.target_object is not None:
        sequence = plan_simulated_grasp_sequence(
            result,
            lift_height_m=args.lift_height,
            lift_duration_s=args.lift_duration,
        )
        lift = sequence.lift
        opening_tilt = sequence.report[
            "maximum_descent_opening_table_tilt_deg"
        ]
        opening_tilt_text = (
            f"{float(opening_tilt):.3f}deg"
            if opening_tilt is not None
            else "not-constrained"
        )
        print(
            "Grasp sequence gate: PASS; "
            f"mode={result.report['grasp_mode']}, "
            f"visual_radius={sequence.visual_radius_m:.3f}m, "
            f"preopen={sequence.preopen_position:.3f}, "
            f"grip={sequence.grip_position:.3f}, "
            f"lift={lift.duration_s:.3f}s, "
            f"hold={sequence.lifted_hold_duration_s:.3f}s, "
            f"lower={sequence.report['controlled_lower_duration_s']:.3f}s, "
            f"return={sequence.return_to_default.duration_s:.3f}s, "
            f"opening_table_tilt={opening_tilt_text}"
        )
        print(
            f"Phases: fully open for {sequence.open_duration_s:.1f}s before "
            "arm motion -> robot-side raised safe staging -> 3cm above object "
            "top -> vertical descend -> radius close -> grasp -> lift -> "
            "hold -> controlled lower -> release -> open-hand retreat -> "
            "close at pregrasp -> default"
        )
        result.report["simulated_grasp_sequence"] = sequence.report
    elif args.lift_trajectory is not None:
        lift = plan_lift_trajectory(
            result,
            lift_height_m=args.lift_height,
            lift_duration_s=args.lift_duration,
        )
    else:
        lift = None
    if args.lift_trajectory is not None:
        assert lift is not None
        lift_trajectory = args.lift_trajectory.expanduser().resolve()
        lift_report = (
            args.lift_report.expanduser().resolve()
            if args.lift_report is not None
            else lift_trajectory.with_name(
                lift_trajectory.stem + "_report.json"
            )
        )
        lift.write(lift_trajectory, lift_report)
        print(f"Lift trajectory: {lift_trajectory}")
        print(f"Lift report: {lift_report}")
        print(
            "Lift gate: PASS; "
            f"frames={len(lift.times)}, duration={lift.duration_s:.3f}s, "
            f"height={args.lift_height:.3f}m, "
            f"final_error={lift.report['final_position_error_m']:.6f}m"
        )
        if lift.report.get("nearest_ik_fallback_used"):
            print(
                "Lift IK nearest-point fallback: ACCEPTED after "
                "collision/edge checks; waypoints="
                f"{lift.report['nearest_ik_fallback_count']}, "
                "maximum_position_error="
                f"{lift.report['nearest_ik_fallback_maximum_error_m']:.6f}m, "
                f"limit={lift.report['nearest_ik_fallback_limit_m']:.3f}m"
            )
    if args.return_trajectory is not None:
        if sequence is None:
            raise RuntimeError(
                "return trajectory requires a selected visual target and complete "
                "grasp sequence"
            )
        return_trajectory = args.return_trajectory.expanduser().resolve()
        return_report = (
            args.return_report.expanduser().resolve()
            if args.return_report is not None
            else return_trajectory.with_name(
                return_trajectory.stem + "_report.json"
            )
        )
        sequence.return_to_default.write(return_trajectory, return_report)
        print(f"Return trajectory: {return_trajectory}")
        print(f"Return report: {return_report}")
        print(
            "Return gate: PASS; "
            f"frames={len(sequence.return_to_default.times)}, "
            f"duration={sequence.return_to_default.duration_s:.3f}s"
        )
    result.write(trajectory, report)
    return sequence if sequence is not None else result


def _validate_trajectory_profile(path: Path, profile: RobotProfile) -> None:
    """Backward-compatible name for the shared animation provenance gate."""

    validate_animation_trajectory_source(path, profile)


def _animation(args: argparse.Namespace, profile: RobotProfile) -> int:
    existing_animation = args.animation is not None
    if existing_animation:
        animation_path = args.animation.expanduser().resolve()
        if not animation_path.is_file():
            raise TrajectoryValidationError(
                f"existing animation CSV does not exist: {animation_path}"
            )
    else:
        trajectory_path = args.trajectory.expanduser().resolve()
        if (
            args.target is not None
            or args.vision_result is not None
            or args.no_vision
            or args.demo_scene
            or not trajectory_path.is_file()
        ):
            print("Running the simulation gate before animation conversion.")
            _plan(args, profile)
        _validate_trajectory_profile(trajectory_path, profile)
        trajectory = load_trajectory(
            trajectory_path,
            maximum_allowed_velocity=profile.maximum_velocity_rad_s + 1e-6,
        )
        animation = build_mc_animation(
            trajectory,
            speed_scale=args.speed_scale,
            maximum_output_velocity=profile.maximum_velocity_rad_s,
        )
        animation_path = write_mc_animation_csv(animation, DEFAULT_ANIMATION)
    info = validate_mc_animation_csv(
        animation_path,
        maximum_velocity=profile.maximum_velocity_rad_s,
    )
    print(f"Robot: {profile.name} ({profile.arm_dof} DoF/arm)")
    print(
        f"Animation: {animation_path} "
        f"({'existing replay' if existing_animation else 'generated from verified plan'})"
    )
    print(
        "Animation gate: PASS; "
        f"frames={info.frame_count}, duration={info.duration_s:.3f}s, "
        f"max_velocity={info.maximum_arm_velocity:.3f}rad/s"
    )
    if not args.execute:
        print("Live robot contact: disabled (add --execute explicitly).")
        return 0
    backend = ROOT / "tools" / "animation_backend.sh"
    command = [
        str(backend),
        "--animation",
        str(animation_path),
    ]
    if args.yes:
        command.append("--yes")
    return subprocess.run(command, cwd=ROOT, check=False).returncode


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)
    try:
        profile = get_robot_profile(args.robot)
        if args.mode == "sim":
            try:
                result = _plan(args, profile)
            except RuntimeError as planning_error:
                if args.headless:
                    raise
                try:
                    vision_result = _resolve_vision_result(args)
                except RuntimeError:
                    raise planning_error
                print(
                    f"Planning failed; opening the static MuJoCo scene: {planning_error}",
                    file=sys.stderr,
                )
                preview_scene(
                    profile,
                    vision_result=vision_result,
                    robot_urdf=args.robot_urdf,
                )
                raise planning_error
            if not args.headless:
                if isinstance(result, SimulatedGraspSequence):
                    replay_grasp_sequence(result)
                else:
                    replay_viewer(result)
            return 0
        return _animation(args, profile)
    except (OSError, RuntimeError, ValueError, TrajectoryValidationError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
