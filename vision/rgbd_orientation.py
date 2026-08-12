#!/usr/bin/env python3
"""Rotate a stored RGB-D frame while keeping its camera metadata consistent."""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
from typing import Any

import cv2
import numpy as np


VALID_ROTATIONS = (0, 180)
MOUNT_ROTATIONS = {"upright": 0, "inverted": 180}


def stored_rotation_deg(camera: dict[str, Any]) -> int:
    """Return the declared stored-image rotation, assuming legacy frames are 0°."""
    orientation = camera.get("image_orientation", {})
    rotation = orientation.get("rotation_deg", 0)
    if isinstance(rotation, bool) or not isinstance(rotation, (int, float)):
        raise ValueError("camera.json image_orientation.rotation_deg must be 0 or 180")
    if float(rotation) not in VALID_ROTATIONS:
        raise ValueError("camera.json image_orientation.rotation_deg must be 0 or 180")
    return int(rotation)


def calibrated_rotation_deg(
    calibration: dict[str, Any], *, default: int | None = None
) -> int:
    """Return and validate the stored-image rotation for one camera mount."""
    model_values = calibration.get("model_values", {})
    if not isinstance(model_values, dict):
        raise ValueError("calibration model_values must be an object")
    rotation = model_values.get("capture_image_rotation_deg", default)
    if isinstance(rotation, bool) or not isinstance(rotation, (int, float)):
        raise ValueError(
            "model_values.capture_image_rotation_deg must be 0 or 180"
        )
    if float(rotation) not in VALID_ROTATIONS:
        raise ValueError(
            "model_values.capture_image_rotation_deg must be 0 or 180"
        )
    rotation_deg = int(rotation)

    mount_orientation = model_values.get("camera_mount_orientation")
    if mount_orientation is not None:
        if (
            not isinstance(mount_orientation, str)
            or mount_orientation not in MOUNT_ROTATIONS
        ):
            raise ValueError(
                "model_values.camera_mount_orientation must be upright or inverted"
            )
        expected = MOUNT_ROTATIONS[mount_orientation]
        if rotation_deg != expected:
            raise ValueError(
                "camera mount/profile mismatch: "
                f"{mount_orientation} requires capture_image_rotation_deg={expected}, "
                f"found {rotation_deg}"
            )
    return rotation_deg


def resolve_camera_transform(
    calibration: dict[str, Any], camera: dict[str, Any]
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Adapt a calibrated transform to the stored 0/180-degree representation."""
    nominal = np.asarray(calibration["T_mujoco_camera_nominal"], dtype=np.float64)
    if nominal.shape != (4, 4):
        raise ValueError("T_mujoco_camera_nominal must be 4x4")
    actual_rotation_deg = stored_rotation_deg(camera)
    calibration_rotation_deg = calibrated_rotation_deg(
        calibration, default=actual_rotation_deg
    )
    selection_mode = camera.get("image_orientation", {}).get("selection_mode")
    auto_mount_hypothesis = selection_mode == "auto"
    # In table-aware auto mode, 0° and 180° are competing physical mount
    # hypotheses.  Each hypothesis represents the optical frame in the same
    # canonical upright axes used by the nominal transform, so do not apply a
    # second relative half-turn from the machine profile.  Explicit/calibrated
    # modes retain the strict profile-relative behavior.
    transform_reference_rotation_deg = (
        actual_rotation_deg
        if auto_mount_hypothesis
        else calibration_rotation_deg
    )

    relative_rotation_deg = (
        transform_reference_rotation_deg - actual_rotation_deg
    ) % 360
    effective = nominal.copy()
    if relative_rotation_deg == 180:
        camera_half_turn = np.diag([-1.0, -1.0, 1.0, 1.0])
        effective = nominal @ camera_half_turn
    elif relative_rotation_deg != 0:
        raise ValueError("Only relative image rotations of 0 or 180 are supported")
    report = {
        "calibration_rotation_deg": calibration_rotation_deg,
        "stored_rotation_deg": actual_rotation_deg,
        "relative_transform_rotation_deg": relative_rotation_deg,
        "adjusted": relative_rotation_deg != 0,
    }
    if auto_mount_hypothesis:
        report["auto_mount_hypothesis"] = True
        report["transform_reference_rotation_deg"] = (
            transform_reference_rotation_deg
        )
    mount_orientation = calibration.get("model_values", {}).get(
        "camera_mount_orientation"
    )
    if mount_orientation is not None:
        report["camera_mount_orientation"] = mount_orientation
    return nominal, effective, report


def _rotate_intrinsics_180(intrinsics: dict[str, Any]) -> None:
    width = int(intrinsics["width"])
    height = int(intrinsics["height"])
    intrinsics["cx"] = (width - 1.0) - float(intrinsics["cx"])
    intrinsics["cy"] = (height - 1.0) - float(intrinsics["cy"])


def _rotate_distortion_180(distortion: dict[str, Any]) -> None:
    # Radial terms are unchanged by a half-turn. Tangential distortion changes
    # sign because both normalized image axes are reversed.
    for name in ("p1", "p2"):
        if name in distortion:
            distortion[name] = -float(distortion[name])


def _rotate_camera_block_180(block: dict[str, Any]) -> None:
    intrinsics = block.get("intrinsics")
    if isinstance(intrinsics, dict):
        _rotate_intrinsics_180(intrinsics)
    distortion = block.get("distortion")
    if isinstance(distortion, dict):
        _rotate_distortion_180(distortion)
    source_intrinsics = block.get("source_intrinsics")
    if isinstance(source_intrinsics, dict):
        _rotate_intrinsics_180(source_intrinsics)
    source_distortion = block.get("source_distortion")
    if isinstance(source_distortion, dict):
        _rotate_distortion_180(source_distortion)


def rotate_rgbd_frame_180(
    color: np.ndarray,
    depth: np.ndarray,
    camera: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Rotate registered RGB/depth and all represented intrinsics by 180°."""
    if color.shape[:2] != depth.shape:
        raise ValueError(
            f"Color/depth shape mismatch before rotation: {color.shape[:2]} vs {depth.shape}"
        )
    updated = copy.deepcopy(camera)
    previous_rotation = stored_rotation_deg(updated)
    for name in ("color", "depth"):
        block = updated.get(name)
        if isinstance(block, dict):
            _rotate_camera_block_180(block)
    updated["image_orientation"] = {
        **updated.get("image_orientation", {}),
        "rotation_deg": (previous_rotation + 180) % 360,
        "reason": "selected by graspV2 orientation handling",
        "applied_to": ["color.png", "depth.png", "camera intrinsics"],
    }
    return (
        cv2.rotate(color, cv2.ROTATE_180),
        cv2.rotate(depth, cv2.ROTATE_180),
        updated,
    )


def set_selection_metadata(
    camera: dict[str, Any], mode: str, attempt: int
) -> dict[str, Any]:
    updated = copy.deepcopy(camera)
    orientation = updated.setdefault("image_orientation", {})
    orientation.setdefault("rotation_deg", stored_rotation_deg(updated))
    orientation["selection_mode"] = mode
    orientation["attempt"] = attempt
    return updated


def _write_png_atomic(path: Path, image: np.ndarray) -> None:
    temporary = path.with_name(f".{path.stem}.orientation{path.suffix}")
    if not cv2.imwrite(str(temporary), image):
        raise RuntimeError(f"Failed to write {temporary}")
    os.replace(temporary, path)


def _write_json_atomic(path: Path, document: dict[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.orientation")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def update_stored_frame(
    frame_dir: Path,
    *,
    target_deg: int | None,
    toggle: bool,
    selection_mode: str,
    attempt: int,
) -> int:
    color_path = frame_dir / "color.png"
    depth_path = frame_dir / "depth.png"
    metadata_path = frame_dir / "camera.json"
    color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    if color is None or depth is None:
        raise RuntimeError(f"Failed to load RGB-D frame from {frame_dir}")
    with metadata_path.open("r", encoding="utf-8") as handle:
        camera = json.load(handle)

    current = stored_rotation_deg(camera)
    should_rotate = toggle or (target_deg is not None and current != target_deg)
    if should_rotate:
        color, depth, camera = rotate_rgbd_frame_180(color, depth, camera)
        _write_png_atomic(color_path, color)
        _write_png_atomic(depth_path, depth)
    camera = set_selection_metadata(camera, selection_mode, attempt)
    _write_json_atomic(metadata_path, camera)
    return stored_rotation_deg(camera)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame-dir", type=Path)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--print-calibrated-rotation", action="store_true")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--target-deg", type=int, choices=VALID_ROTATIONS)
    action.add_argument("--toggle", action="store_true")
    parser.add_argument(
        "--selection-mode",
        choices=("calibrated", "auto", "explicit", "auto-unverified"),
    )
    parser.add_argument("--attempt", type=int, default=1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.print_calibrated_rotation:
        if args.calibration is None:
            raise SystemExit(
                "--print-calibrated-rotation requires --calibration"
            )
        with args.calibration.expanduser().resolve().open(
            "r", encoding="utf-8"
        ) as handle:
            calibration = json.load(handle)
        print(calibrated_rotation_deg(calibration))
        return 0
    if args.frame_dir is None or args.selection_mode is None:
        raise SystemExit(
            "stored-frame updates require --frame-dir and --selection-mode"
        )
    rotation = update_stored_frame(
        args.frame_dir.resolve(),
        target_deg=args.target_deg,
        toggle=args.toggle,
        selection_mode=args.selection_mode,
        attempt=args.attempt,
    )
    print(f"Prepared stored RGB-D rotation: {rotation} deg ({args.selection_mode})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
