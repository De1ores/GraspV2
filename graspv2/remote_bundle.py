"""Build the small, checksum-bound execution bundle sent to the X2 Orin."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Sequence

from .edge_manifest import (
    ANIMATION_FILENAME,
    BUNDLE_KIND,
    SCHEMA_VERSION,
    load_edge_manifest,
    sha256_file,
)
from .grasp_sequence import (
    VisualVerificationConfig,
    load_grasp_plan_metadata,
    load_visual_observation,
    validate_trajectory_continuity,
    verify_initial_observation,
)
from .mc_animation import (
    build_mc_grasp_animation,
    validate_animation_trajectory_source,
    validate_mc_animation_csv,
    write_mc_animation_csv,
)
from .robot_profiles import RobotProfile, get_robot_profile
from .trajectory import JointTrajectory, load_trajectory


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--approach-trajectory", type=Path, required=True)
    parser.add_argument("--lift-trajectory", type=Path, required=True)
    parser.add_argument("--return-trajectory", type=Path, required=True)
    parser.add_argument("--initial-vision", type=Path, required=True)
    parser.add_argument("--target-class", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser


def _return_error(
    profile: RobotProfile, trajectory: JointTrajectory
) -> float:
    default_by_name = dict(
        zip(profile.arm_pos_order, profile.mc_start_arm_pos())
    )
    return max(
        abs(value - default_by_name[name])
        for name, value in zip(
            trajectory.joint_names,
            trajectory.positions[-1],
        )
    )


def build_remote_bundle(args: argparse.Namespace) -> Path:
    """Validate local artifacts and create the only files the robot consumes."""

    profile = get_robot_profile("ultra")
    approach = load_trajectory(
        args.approach_trajectory,
        maximum_allowed_velocity=profile.maximum_velocity_rad_s + 1e-6,
    )
    lift = load_trajectory(
        args.lift_trajectory,
        maximum_allowed_velocity=profile.maximum_velocity_rad_s + 1e-6,
    )
    return_to_default = load_trajectory(
        args.return_trajectory,
        maximum_allowed_velocity=profile.maximum_velocity_rad_s + 1e-6,
    )
    validate_trajectory_continuity(approach, lift)
    validate_trajectory_continuity(lift, return_to_default)
    endpoint_error = _return_error(profile, return_to_default)
    if endpoint_error > 1e-6:
        raise ValueError(
            "return trajectory does not end at the MC default pose: "
            f"error={endpoint_error:.9f} rad"
        )
    metadata = load_grasp_plan_metadata(
        args.approach_trajectory,
        args.lift_trajectory,
        args.return_trajectory,
    )
    if metadata.robot_profile != profile.name or metadata.side != "right":
        raise ValueError("execution plan must target the Ultra right OmniPicker")
    validate_animation_trajectory_source(args.approach_trajectory, profile)
    observation = load_visual_observation(args.initial_vision, args.target_class)
    verify_initial_observation(
        observation,
        metadata.object_center_world_m,
        VisualVerificationConfig().validate(),
    )

    animation = build_mc_grasp_animation(
        approach,
        lift,
        return_to_default,
        metadata,
        speed_scale=1.0,
        maximum_output_velocity=profile.maximum_velocity_rad_s,
    )
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    animation_path = write_mc_animation_csv(
        animation,
        output_dir / ANIMATION_FILENAME,
    )
    info = validate_mc_animation_csv(
        animation_path,
        maximum_velocity=profile.maximum_velocity_rad_s,
    )

    source_files = {
        "approach_trajectory": args.approach_trajectory,
        "lift_trajectory": args.lift_trajectory,
        "return_trajectory": args.return_trajectory,
        "initial_vision": args.initial_vision,
    }
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "kind": BUNDLE_KIND,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "robot": {"profile": profile.name, "arm_side": metadata.side},
        "target_class": args.target_class,
        "animation": {
            "file": ANIMATION_FILENAME,
            "sha256": sha256_file(animation_path),
            "duration_s": info.duration_s,
            "maximum_arm_velocity_rad_s": info.maximum_arm_velocity,
            "initial_gripper": {
                "position": animation.initial_gripper_position,
                "duration_s": animation.initial_gripper_duration_s,
            },
            "gripper_events": [
                asdict(event) for event in animation.gripper_events
            ],
            "return_path_enabled": animation.return_path_enabled,
        },
        "local_safety_evidence": {
            "official_ik": True,
            "collision_checked": True,
            "initial_vision_checked": True,
            "return_endpoint_error_rad": endpoint_error,
            "source_sha256": {
                label: sha256_file(path.expanduser().resolve())
                for label, path in source_files.items()
            },
        },
        "execution_limitations": {
            "atomic_mc_animation": True,
            "mid_motion_visual_gates": False,
        },
    }
    manifest_path = output_dir / "manifest.json"
    temporary = manifest_path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(manifest_path)
    load_edge_manifest(manifest_path)
    return manifest_path


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if not args.target_class.strip():
        parser.error("--target-class must be non-empty")
    try:
        manifest_path = build_remote_bundle(args)
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(str(error))
    manifest = load_edge_manifest(manifest_path)
    print(f"Edge execution bundle: {manifest_path.parent}")
    print(f"Animation SHA-256: {manifest.animation_sha256}")
    print(
        f"Animation: {manifest.duration_s:.3f} s, "
        f"{len(manifest.gripper_events)} OmniPicker events"
    )
    print(
        "Execution model: atomic MC animation with an embedded safe return; "
        "no mid-motion visual gates"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
