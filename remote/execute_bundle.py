#!/usr/bin/env python3
"""Validate, stage and execute one local-compute GraspV2 edge bundle."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import sys
from typing import Sequence


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from graspv2.edge_manifest import load_edge_manifest, sha256_file  # noqa: E402


ROBOT_ANIMATION_PATH = Path("/tmp/graspv2_mc_grasp_animation.csv")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--preflight", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-control-authority", action="store_true")
    return parser


def _stage_animation(source: Path, expected_sha256: str) -> Path:
    temporary = ROBOT_ANIMATION_PATH.with_name(
        f".{ROBOT_ANIMATION_PATH.name}.tmp.{os.getpid()}"
    )
    try:
        with source.open("rb") as input_stream, temporary.open("xb") as output_stream:
            shutil.copyfileobj(input_stream, output_stream, length=1024 * 1024)
            output_stream.flush()
            os.fsync(output_stream.fileno())
        temporary.chmod(0o644)
        if sha256_file(temporary) != expected_sha256:
            raise RuntimeError("staged robot animation failed its SHA-256 check")
        os.replace(temporary, ROBOT_ANIMATION_PATH)
    finally:
        if temporary.exists():
            temporary.unlink()
    return ROBOT_ANIMATION_PATH


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.execute and not args.confirm_control_authority:
        parser.error("--execute requires --confirm-control-authority")
    if args.preflight and args.confirm_control_authority:
        parser.error("--confirm-control-authority is only valid with --execute")

    manifest_path = args.bundle.expanduser().resolve() / "manifest.json"
    try:
        manifest = load_edge_manifest(manifest_path)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    print(f"Verified edge manifest: {manifest.source}")
    print(f"Verified animation SHA-256: {manifest.animation_sha256}")
    print(
        "Safety boundary: robot-side AimDK preflight + atomic MC animation; "
        "mid-motion visual verification is unavailable in split mode"
    )

    if args.execute:
        try:
            robot_animation = _stage_animation(
                manifest.animation_path,
                manifest.animation_sha256,
            )
        except (OSError, RuntimeError) as error:
            parser.error(str(error))
    else:
        robot_animation = ROBOT_ANIMATION_PATH

    # Import only after the wrapper sourced the robot's ROS/AimDK overlay.
    try:
        from graspv2 import mc_custom_grasp
    except (ImportError, ModuleNotFoundError) as error:
        parser.error(
            "ROS 2/AimDK Python interfaces are unavailable after environment "
            f"discovery: {error}"
        )

    command = [
        "--animation",
        str(manifest.animation_path),
        "--robot-animation-path",
        str(robot_animation),
        "--omnipicker-sdk",
        str(ROOT / "omnipicker_hand_student.py"),
        "--require-gripper-sdk",
        "--initial-gripper-position",
        f"{manifest.initial_gripper_position:.9f}",
        "--initial-gripper-duration",
        f"{manifest.initial_gripper_duration_s:.9f}",
        "--max-velocity",
        "0.5",
    ]
    for event in manifest.gripper_events:
        command.extend(
            (
                "--gripper-event",
                f"{event.time_s:.9f}:{event.position:.9f}:{event.label}",
            )
        )
    command.append("--execute" if args.execute else "--preflight")
    return mc_custom_grasp.main(command)


if __name__ == "__main__":
    raise SystemExit(main())
