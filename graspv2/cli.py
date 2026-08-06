"""Single public command for simulation planning and MC animation replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

from .mc_animation import (
    build_mc_animation,
    validate_mc_animation_csv,
    write_mc_animation_csv,
)
from .planner import (
    plan_lift_trajectory,
    plan_trajectory,
    preview_scene,
    replay_viewer,
)
from .robot_profiles import PROFILES, RobotProfile, get_robot_profile
from .trajectory import TrajectoryValidationError, load_trajectory


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY = ROOT / "output" / "planned_trajectory.json"
DEFAULT_REPORT = ROOT / "output" / "planning_report.json"
DEFAULT_ANIMATION = ROOT / "output" / "mc_animation.csv"
DEFAULT_VISION_RESULT = ROOT / "output" / "result.json"


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
      使用上次视觉抓取点生成桌子，规划后打开 MuJoCo Viewer。

  ./run.sh --headless
      使用同一桌子和规划，但不打开窗口，适用于 SSH/Orin。

  ./run.sh --robot ultra
      使用同一份上次视觉结果，为 Ultra 规划并打开 Viewer。

  ./run.sh --robot youth --vision-result output/result.json
      显式指定另一份视觉结果；语义与默认路径相同。

  ./run.sh --target 0.38 -0.30 0.92
      保留默认视觉桌面，仅用手工世界坐标覆盖视觉抓取点。

  ./run.sh --no-vision
      不读取视觉 JSON，不生成桌面障碍，改用内置可达测试目标。

  ./run.sh --robot youth --mode animation
      把已验证轨迹转换为 CSV；完全离线，不连接机器人。

  ./run.sh --robot youth --mode animation --animation /path/to/action.csv
      校验并准备重放已有 CSV，不重新规划或覆盖该文件。

  ./run.sh --robot youth --mode animation --execute
      真机只读预检、上传校验并在再次输入 RUN 后播放。

坐标约定：米；+X 向机器人前方，+Y 向机器人左侧，+Z 向上。
退出码：0 成功；1 规划/校验/执行失败；2 命令行参数错误。""",
    )
    parser.add_argument(
        "--robot",
        choices=tuple(PROFILES),
        default="youth",
        help=(
            "机器人配置（默认 youth）；youth=每臂5轴，ultra=每臂7轴，"
            "future_upper 尚未拿到比赛机模型，当前会安全拒绝运行"
        ),
    )
    parser.add_argument(
        "--mode",
        choices=("sim", "animation"),
        default="sim",
        help="运行模式（默认 sim）；sim=规划仿真，animation=转换或重放 CSV",
    )
    parser.add_argument(
        "--side",
        choices=("auto", "left", "right"),
        default="auto",
        help=(
            "规划手臂（默认 auto）；优先使用目标同侧手臂，规划失败时自动尝试另一只手"
        ),
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
            "实际抓取 TCP 的目标世界坐标，单位米；默认取视觉抓取点；"
            "与视觉结果同用时只覆盖抓取点，仍保留视觉桌面障碍"
        ),
    )
    vision_group = parser.add_mutually_exclusive_group()
    vision_group.add_argument(
        "--vision-result",
        type=Path,
        help=(
            f"视觉结果 JSON（默认 {DEFAULT_VISION_RESULT}），读取 "
            "grasp_point_mujoco_m 和桌面碰撞盒"
        ),
    )
    vision_group.add_argument(
        "--no-vision",
        action="store_true",
        help="禁用默认视觉结果和桌面障碍；无 --target 时使用内置可达测试目标",
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
        help="同时生成从抓取点向桌面法向抬升的独立碰撞验证轨迹",
    )
    parser.add_argument(
        "--lift-report",
        type=Path,
        help="抬升轨迹验证报告；默认使用 lift trajectory 同目录下的 report 文件",
    )
    parser.add_argument(
        "--lift-height",
        type=float,
        default=0.10,
        help="视觉验证抓取使用的抬升距离，单位米（默认 0.10）",
    )
    parser.add_argument(
        "--lift-duration",
        type=float,
        default=2.0,
        help="抬升动作时长，单位秒（默认 2.0）",
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
        default=0.075,
        help="预抓取点沿桌面法向离目标的距离，单位米（默认 0.075，范围 0.01~0.30）",
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
        args.target is not None or args.vision_result is not None or args.no_vision
    ):
        parser.error("an existing --animation cannot be combined with planning inputs")
    if args.lift_report is not None and args.lift_trajectory is None:
        parser.error("--lift-report requires --lift-trajectory")
    if args.lift_trajectory is not None and args.mode != "sim":
        parser.error("--lift-trajectory is only valid with --mode sim")


def _resolve_vision_result(args: argparse.Namespace) -> Path | None:
    if args.no_vision:
        return None
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


def _automatic_side_order(
    args: argparse.Namespace,
    vision_result: Path | None,
) -> tuple[str, ...]:
    """Return the requested side or both sides in deterministic preference order."""
    if args.side != "auto":
        return (args.side,)
    target = args.target
    if target is None and vision_result is not None:
        try:
            document = json.loads(vision_result.read_text(encoding="utf-8"))
            raw_target = document.get("grasp_point_mujoco_m")
            if isinstance(raw_target, list) and len(raw_target) == 3:
                target = raw_target
        except (OSError, json.JSONDecodeError):
            # plan_trajectory will emit the authoritative input error.  The
            # fallback order remains deterministic if this lightweight hint
            # cannot be read here.
            target = None
    preferred = "left" if target is not None and float(target[1]) >= 0.0 else "right"
    alternate = "right" if preferred == "left" else "left"
    return preferred, alternate


def _plan_with_side_fallback(
    args: argparse.Namespace,
    profile: RobotProfile,
    vision_result: Path | None,
):
    sides = _automatic_side_order(args, vision_result)
    if len(sides) == 2:
        print(f"Automatic arm order: {sides[0]} -> {sides[1]}")
    failures: list[tuple[str, str]] = []
    for index, side in enumerate(sides):
        if index:
            print(f"Retrying the same target with the {side} arm.")
        try:
            result = plan_trajectory(
                profile,
                side=side,
                target_world_xyz=args.target,
                vision_result=vision_result,
                robot_urdf=args.robot_urdf,
                table_clearance_m=args.table_clearance,
                approach_distance_m=args.approach_distance,
            )
            if len(sides) == 2:
                print(f"Automatic arm selection: {side}")
            return result
        except RuntimeError as error:
            failures.append((side, str(error)))
            if index + 1 < len(sides):
                print(f"{side.capitalize()} arm planning failed: {error}", file=sys.stderr)
    if len(failures) == 1:
        raise RuntimeError(failures[0][1])
    details = "; ".join(f"{side}: {message}" for side, message in failures)
    raise RuntimeError(f"automatic arm planning failed for both arms; {details}")


def _plan(args: argparse.Namespace, profile: RobotProfile):
    vision_result = _resolve_vision_result(args)
    print(
        f"Vision obstacle: {vision_result}"
        if vision_result is not None
        else "Vision obstacle: disabled (--no-vision)"
    )
    result = _plan_with_side_fallback(args, profile, vision_result)
    trajectory = args.trajectory.expanduser().resolve()
    report = args.report.expanduser().resolve()
    result.write(trajectory, report)
    print(f"Robot: {profile.name} ({profile.arm_dof} DoF/arm)")
    print("IK: official x2_ik_sdk")
    visual_urdf = result.report["robot_visual_urdf"]
    print(
        f"Robot visual URDF: {visual_urdf}"
        if visual_urdf is not None
        else "Robot visual URDF: not found (collision proxies only)"
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
    if args.lift_trajectory is not None:
        lift = plan_lift_trajectory(
            result,
            lift_height_m=args.lift_height,
            lift_duration_s=args.lift_duration,
        )
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
    return result


def _read_trajectory_document(path: Path) -> dict[str, object]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise TrajectoryValidationError(f"cannot read trajectory metadata: {error}") from error
    if not isinstance(document, dict):
        raise TrajectoryValidationError("trajectory root must be an object")
    return document


def _validate_trajectory_profile(path: Path, profile: RobotProfile) -> None:
    document = _read_trajectory_document(path)
    value = document.get("robot_profile")
    if value is not None and not isinstance(value, str):
        raise TrajectoryValidationError("trajectory robot_profile must be a string")
    declared = value
    if declared is not None and declared != profile.name:
        raise TrajectoryValidationError(
            f"trajectory was generated for {declared!r}, not {profile.name!r}"
        )
    planning = document.get("planning")
    if not isinstance(planning, dict):
        raise TrajectoryValidationError("trajectory has no planning verification")
    if planning.get("verified_collision_free") is not True:
        raise TrajectoryValidationError("trajectory is not verified collision-free")
    if planning.get("ik_backend") != "x2_ik_sdk.X2ArmIKSolver":
        raise TrajectoryValidationError("trajectory was not generated by official IK")


def _animation(args: argparse.Namespace, profile: RobotProfile) -> int:
    if not profile.animation_supported:
        raise RuntimeError(f"{profile.name} has no verified animation backend")
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
            or not trajectory_path.is_file()
        ):
            print("Running the simulation gate before animation conversion.")
            _plan(args, profile)
        _validate_trajectory_profile(trajectory_path, profile)
        trajectory = load_trajectory(
            trajectory_path,
            maximum_allowed_velocity=profile.maximum_velocity_rad_s + 1e-6,
        )
        unavailable = sorted(
            set(trajectory.joint_names) & set(profile.absent_animation_joints)
        )
        if unavailable:
            raise TrajectoryValidationError(
                f"{profile.name} trajectory commands unavailable joints: "
                + ", ".join(unavailable)
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
        "--robot",
        profile.name,
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
                replay_viewer(result)
            return 0
        return _animation(args, profile)
    except (OSError, RuntimeError, ValueError, TrajectoryValidationError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
