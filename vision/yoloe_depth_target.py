#!/usr/bin/env python3
"""Estimate one YOLOE grasp point and the table plane in MuJoCo world coordinates."""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
THIRD_PARTY = []
use_bundled_python = os.environ.get("GRASPV2_USE_BUNDLED_PYTHON", "auto").lower()
if use_bundled_python not in {"0", "false", "no"} and (
    use_bundled_python != "auto" or platform.machine() in {"x86_64", "amd64"}
):
    THIRD_PARTY.append(ROOT / "third_party/python")
THIRD_PARTY.append(ROOT / "third_party/ultralytics_clip")
for directory in reversed(THIRD_PARTY):
    if directory.exists():
        sys.path.insert(0, str(directory))
os.environ.setdefault("MPLCONFIGDIR", "/tmp/graspv2-matplotlib")

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from ultralytics import YOLO  # noqa: E402


DEFAULT_CLASSES = [
    "bottle",
    "cup",
    "mug",
    "bowl",
    "box",
    "can",
    "remote control",
    "screwdriver",
    "keyboard",
    "game controller",
    "pen",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame-dir", type=Path, default=ROOT / "output")
    parser.add_argument("--output-dir", type=Path, help="Default: FRAME_DIR")
    parser.add_argument("--weights", type=Path, default=ROOT / "yoloe-26s-seg.pt")
    parser.add_argument("--classes", nargs="+", default=DEFAULT_CLASSES)
    parser.add_argument("--target-class", help="Select the highest confidence instance of this class")
    parser.add_argument("--detection-index", type=int, help="Select an exact detection index instead")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--device", default="auto", help="auto, cpu, or a CUDA device such as 0")
    parser.add_argument("--min-depth-m", type=float, default=0.15)
    parser.add_argument("--max-depth-m", type=float, default=2.0)
    parser.add_argument("--mask-erode-px", type=int, default=5)
    parser.add_argument("--mad-multiplier", type=float, default=3.5)
    parser.add_argument("--min-depth-band-m", type=float, default=0.012)
    parser.add_argument("--normal-window-px", type=int, default=35)
    parser.add_argument("--pregrasp-offset-m", type=float, default=0.08)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config/mujoco_camera_calibration.json",
        help="Camera-to-x2_arm_sim MuJoCo world calibration JSON",
    )
    parser.add_argument(
        "--offset-mujoco-m",
        type=float,
        nargs=3,
        metavar=("DX", "DY", "DZ"),
        help="Override the configured final point offset in MuJoCo world axes",
    )
    return parser.parse_args()


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def as_list(value: np.ndarray, digits: int = 6) -> list[float]:
    return [round(float(item), digits) for item in value.reshape(-1)]


def pixels_to_rays(
    pixels_uv: np.ndarray,
    intrinsics: dict[str, float],
    distortion: dict[str, Any],
) -> tuple[np.ndarray, str]:
    """Convert distorted color pixels to unit-z camera rays."""
    fx, fy = float(intrinsics["fx"]), float(intrinsics["fy"])
    cx, cy = float(intrinsics["cx"]), float(intrinsics["cy"])
    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    model = str(distortion.get("model", "none"))
    coefficients = np.array(
        [
            distortion.get("k1", 0.0),
            distortion.get("k2", 0.0),
            distortion.get("p1", 0.0),
            distortion.get("p2", 0.0),
            distortion.get("k3", 0.0),
            distortion.get("k4", 0.0),
            distortion.get("k5", 0.0),
            distortion.get("k6", 0.0),
        ],
        dtype=np.float64,
    )

    points = pixels_uv.astype(np.float64).reshape(-1, 1, 2)
    if model == "kannala_brandt4":
        normalized = cv2.fisheye.undistortPoints(points, camera_matrix, coefficients[:4]).reshape(-1, 2)
        method = "opencv_fisheye_undistort"
    elif model != "none" and np.any(np.abs(coefficients) > 1e-12):
        normalized = cv2.undistortPoints(points, camera_matrix, coefficients).reshape(-1, 2)
        method = "opencv_brown_conrady_undistort"
    else:
        normalized = np.column_stack(((pixels_uv[:, 0] - cx) / fx, (pixels_uv[:, 1] - cy) / fy))
        method = "pinhole_no_distortion"
    return np.column_stack((normalized, np.ones(len(normalized), dtype=np.float64))), method


def transform_point(matrix: np.ndarray, point: np.ndarray) -> np.ndarray:
    return (matrix @ np.append(point, 1.0))[:3]


def transform_vector(matrix: np.ndarray, vector: np.ndarray) -> np.ndarray:
    transformed = matrix[:3, :3] @ vector
    return transformed / np.linalg.norm(transformed)


def select_detection(detections: list[dict[str, Any]], args: argparse.Namespace) -> int:
    if not detections:
        raise RuntimeError("YOLOE returned no detections; lower --conf or adjust --classes")
    if args.detection_index is not None:
        if not 0 <= args.detection_index < len(detections):
            raise ValueError(f"--detection-index must be in [0, {len(detections) - 1}]")
        return args.detection_index
    candidates = detections
    if args.target_class:
        candidates = [item for item in detections if item["class_name"].casefold() == args.target_class.casefold()]
        if not candidates:
            found = sorted({item["class_name"] for item in detections})
            raise RuntimeError(f"No '{args.target_class}' detected; found: {found}")
    return int(max(candidates, key=lambda item: item["confidence"])["index"])


def estimate_surface_normal(
    points_camera: np.ndarray,
    pixels_uv: np.ndarray,
    center_uv: np.ndarray,
    window_px: int,
    surface_point: np.ndarray,
) -> tuple[np.ndarray, int, np.ndarray]:
    pixel_distance_sq = np.sum((pixels_uv - center_uv) ** 2, axis=1)
    local = points_camera[pixel_distance_sq <= window_px**2]
    if len(local) < 30:
        count = min(len(points_camera), 500)
        local = points_camera[np.argpartition(pixel_distance_sq, count - 1)[:count]]
    covariance = np.cov(local - np.mean(local, axis=0), rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    normal = eigenvectors[:, np.argmin(eigenvalues)]
    normal /= np.linalg.norm(normal)
    if np.dot(normal, -surface_point) < 0.0:
        normal = -normal
    return normal, len(local), eigenvalues


def clear_previous_output(output_dir: Path) -> None:
    """Keep one result frame while preserving the RGB-D inputs."""
    generated_names = (
        "annotated.png",
        "selected_mask.png",
        "object_points.npz",
        "selected_object_points.npz",
        "result.json",
        "target_point.json",
    )
    for name in generated_names:
        path = output_dir / name
        if path.is_file():
            path.unlink()
    for path in output_dir.glob("mask_*.png"):
        if path.is_file():
            path.unlink()


def rotation_matrix_to_quaternion_wxyz(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a normalized wxyz quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [0.25 * scale, (rotation[2, 1] - rotation[1, 2]) / scale,
             (rotation[0, 2] - rotation[2, 0]) / scale, (rotation[1, 0] - rotation[0, 1]) / scale]
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [(rotation[2, 1] - rotation[1, 2]) / scale, 0.25 * scale,
                 (rotation[0, 1] + rotation[1, 0]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale]
            )
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [(rotation[0, 2] - rotation[2, 0]) / scale, (rotation[0, 1] + rotation[1, 0]) / scale,
                 0.25 * scale, (rotation[1, 2] + rotation[2, 1]) / scale]
            )
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.array(
                [(rotation[1, 0] - rotation[0, 1]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale,
                 (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale]
            )
    return quaternion / np.linalg.norm(quaternion)


def rotation_matrix_to_rpy_degrees(rotation: np.ndarray) -> np.ndarray:
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) > 1e-9:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.degrees([roll, pitch, yaw])


def select_connected_plane_component(
    pixels_uv: np.ndarray,
    inliers: np.ndarray,
    image_shape: tuple[int, int],
    sample_stride_px: int,
    anchor_uv: np.ndarray | None,
    settings: dict[str, Any],
) -> tuple[np.ndarray, dict[str, Any]]:
    """Keep one image-connected patch of a fitted 3D plane.

    RANSAC alone cannot distinguish the tabletop from a disconnected coplanar
    surface.  Connecting the sampled inliers in image space removes those
    background islands before the rectangle is estimated.
    """
    inlier_pixels = np.rint(pixels_uv[inliers]).astype(np.int32)
    sparse = np.zeros(image_shape, dtype=np.uint8)
    sparse[inlier_pixels[:, 1], inlier_pixels[:, 0]] = 255
    connect_px = max(
        2 * sample_stride_px + 1,
        int(settings.get("component_connect_px", 9)) | 1,
    )
    connected = cv2.morphologyEx(
        sparse,
        cv2.MORPH_CLOSE,
        np.ones((connect_px, connect_px), dtype=np.uint8),
    )
    connected = cv2.dilate(
        connected,
        np.ones((2 * sample_stride_px + 1, 2 * sample_stride_px + 1), dtype=np.uint8),
    )
    label_count, labels, _, _ = cv2.connectedComponentsWithStats(connected, connectivity=8)
    if label_count <= 1:
        raise RuntimeError("Table-plane inliers do not form an image-connected region")

    point_labels = labels[inlier_pixels[:, 1], inlier_pixels[:, 0]]
    labels_present, counts = np.unique(point_labels[point_labels > 0], return_counts=True)
    if len(labels_present) == 0:
        raise RuntimeError("Table-plane component labeling produced no foreground points")
    largest_index = int(np.argmax(counts))
    chosen_label = int(labels_present[largest_index])
    anchor_distance_px: float | None = None
    if anchor_uv is not None:
        largest_count = int(counts[largest_index])
        minimum_count = max(30, int(largest_count * 0.20))
        maximum_distance = float(settings.get("component_anchor_max_distance_px", 160.0))
        candidates: list[tuple[float, int]] = []
        for label, count in zip(labels_present, counts):
            if int(count) < minimum_count:
                continue
            component_pixels = inlier_pixels[point_labels == label].astype(np.float64)
            distance = float(
                np.sqrt(np.min(np.sum((component_pixels - anchor_uv) ** 2, axis=1)))
            )
            if distance <= maximum_distance:
                candidates.append((distance, int(label)))
        if candidates:
            anchor_distance_px, chosen_label = min(candidates)

    selected = np.zeros_like(inliers)
    selected_indices = np.flatnonzero(inliers)
    selected[selected_indices[point_labels == chosen_label]] = True
    selected_count = int(np.count_nonzero(selected))
    if selected_count < int(settings.get("minimum_component_points", 150)):
        raise RuntimeError(
            f"Connected tabletop patch contains too few points: {selected_count}"
        )
    return selected, {
        "component_count": int(label_count - 1),
        "selected_component_point_count": selected_count,
        "anchor_distance_px": (
            round(anchor_distance_px, 3) if anchor_distance_px is not None else None
        ),
        "connect_kernel_px": connect_px,
    }


def fit_oriented_rectangle(
    points_xy: np.ndarray,
    extent_quantiles: tuple[float, float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Fit a robust rectangle in plane coordinates using OpenCV edge orientation."""
    if len(points_xy) < 30:
        raise RuntimeError("Too few points for tabletop rectangle fitting")
    rectangle = cv2.minAreaRect(points_xy.astype(np.float32))
    box = cv2.boxPoints(rectangle).astype(np.float64)
    edges = (box[1] - box[0], box[3] - box[0])
    axes = [edge / np.linalg.norm(edge) for edge in edges]
    # Make local X the rectangle edge closest to MuJoCo +X.  This keeps the
    # quaternion stable between captures while still following the table edge.
    axis_x = max(axes, key=lambda axis: abs(float(axis[0])))
    if axis_x[0] < 0.0:
        axis_x = -axis_x
    axis_y = np.array([-axis_x[1], axis_x[0]], dtype=np.float64)
    projected = np.column_stack((points_xy @ axis_x, points_xy @ axis_y))
    q_low, q_high = extent_quantiles
    first_bounds = np.quantile(projected, [q_low, q_high], axis=0)
    robust = np.all(
        (projected >= first_bounds[0]) & (projected <= first_bounds[1]), axis=1
    )
    # Recompute the edge direction after removing the tails so a handful of
    # stray coplanar points cannot rotate the rectangle.
    if int(np.count_nonzero(robust)) >= 30:
        rectangle = cv2.minAreaRect(points_xy[robust].astype(np.float32))
        box = cv2.boxPoints(rectangle).astype(np.float64)
        edges = (box[1] - box[0], box[3] - box[0])
        axes = [edge / np.linalg.norm(edge) for edge in edges]
        axis_x = max(axes, key=lambda axis: abs(float(axis[0])))
        if axis_x[0] < 0.0:
            axis_x = -axis_x
        axis_y = np.array([-axis_x[1], axis_x[0]], dtype=np.float64)
        projected = np.column_stack((points_xy @ axis_x, points_xy @ axis_y))
    bounds = np.quantile(projected, [q_low, q_high], axis=0)
    size = bounds[1] - bounds[0]
    if np.any(size <= 0.0):
        raise RuntimeError(f"Invalid fitted tabletop rectangle size: {size.tolist()}")
    return axis_x, axis_y, bounds, int(np.count_nonzero(robust))


def estimate_table_plane(
    depth_raw: np.ndarray,
    camera: dict[str, Any],
    excluded_mask: np.ndarray,
    matrix: np.ndarray,
    point_offset: np.ndarray,
    settings: dict[str, Any],
    anchor_uv: np.ndarray | None = None,
) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    """Fit one horizontal, connected, rectangular 3D tabletop patch."""
    height, width = depth_raw.shape
    x0f, y0f, x1f, y1f = [float(value) for value in settings["roi_uv_fraction"]]
    x0, x1 = max(0, int(x0f * width)), min(width - 1, int(x1f * width))
    y0, y1 = max(0, int(y0f * height)), min(height - 1, int(y1f * height))
    stride = int(settings["sample_stride_px"])
    rows_grid, cols_grid = np.mgrid[y0 : y1 + 1 : stride, x0 : x1 + 1 : stride]
    rows, cols = rows_grid.reshape(-1), cols_grid.reshape(-1)

    scale_m = float(camera["depth"]["scale_m_per_unit"])
    min_depth, max_depth = [float(value) for value in settings["depth_range_m"]]
    depths = depth_raw[rows, cols].astype(np.float64) * scale_m
    valid = (
        (depth_raw[rows, cols] > 0)
        & (depths >= min_depth)
        & (depths <= max_depth)
        & ~excluded_mask[rows, cols]
    )
    rows, cols, depths = rows[valid], cols[valid], depths[valid]
    if len(rows) < 100:
        raise RuntimeError("Too few valid depth samples for table-plane fitting")

    pixels_uv = np.column_stack((cols, rows)).astype(np.float64)
    rays, _ = pixels_to_rays(pixels_uv, camera["color"]["intrinsics"], camera["color"]["distortion"])
    points_camera = rays * depths[:, None]
    points_mujoco = (matrix[:3, :3] @ points_camera.T).T + matrix[:3, 3] + point_offset

    threshold = float(settings["distance_threshold_m"])
    max_tilt_deg = float(settings.get("max_tilt_from_horizontal_deg", 25.0))
    minimum_normal_z = float(np.cos(np.radians(max_tilt_deg)))
    height_range = tuple(
        float(value)
        for value in settings.get("height_range_mujoco_m", [-float("inf"), float("inf")])
    )
    if len(height_range) != 2 or height_range[0] >= height_range[1]:
        raise ValueError("table_detection.height_range_mujoco_m must be [minimum, maximum]")
    rng = np.random.default_rng(20260729)
    best_inliers: np.ndarray | None = None
    best_count = -1
    for _ in range(int(settings["ransac_iterations"])):
        sample = points_mujoco[rng.choice(len(points_mujoco), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        if normal[2] < 0.0:
            normal = -normal
        if normal[2] < minimum_normal_z:
            continue
        plane_d = -float(np.dot(normal, sample[0]))
        plane_height = -plane_d / normal[2]
        if not height_range[0] <= plane_height <= height_range[1]:
            continue
        inliers = np.abs(points_mujoco @ normal + plane_d) <= threshold
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_count, best_inliers = count, inliers
    if best_inliers is None:
        raise RuntimeError("Table-plane RANSAC did not find a valid model")

    for _ in range(3):
        inlier_points = points_mujoco[best_inliers]
        centroid = np.mean(inlier_points, axis=0)
        _, _, vectors = np.linalg.svd(inlier_points - centroid, full_matrices=False)
        normal = vectors[-1]
        if normal[2] < 0.0:
            normal = -normal
        plane_d = -float(np.dot(normal, centroid))
        residuals = np.abs(points_mujoco @ normal + plane_d)
        best_inliers = residuals <= threshold

    fitted_height = -plane_d / normal[2]
    fitted_tilt_deg = float(np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0))))
    if fitted_tilt_deg > max_tilt_deg:
        raise RuntimeError(
            f"Table plane tilt {fitted_tilt_deg:.2f} deg exceeds {max_tilt_deg:.2f} deg"
        )
    if not height_range[0] <= fitted_height <= height_range[1]:
        raise RuntimeError(
            f"Table plane height {fitted_height:.3f} m is outside {height_range}"
        )

    inlier_count = int(np.count_nonzero(best_inliers))
    inlier_ratio = inlier_count / len(points_mujoco)
    if inlier_ratio < float(settings["min_inlier_ratio"]):
        raise RuntimeError(f"Table plane inlier ratio is too low: {inlier_ratio:.3f}")

    component_inliers, component_report = select_connected_plane_component(
        pixels_uv,
        best_inliers,
        depth_raw.shape,
        stride,
        anchor_uv,
        settings,
    )
    inlier_points = points_mujoco[component_inliers]
    plane_origin = np.mean(inlier_points, axis=0)
    _, _, vectors = np.linalg.svd(inlier_points - plane_origin, full_matrices=False)
    normal = vectors[-1]
    if normal[2] < 0.0:
        normal = -normal
    plane_d = -float(np.dot(normal, plane_origin))
    residuals_inlier = np.abs(inlier_points @ normal + plane_d)
    refined = residuals_inlier <= threshold
    inlier_points = inlier_points[refined]
    residuals_inlier = residuals_inlier[refined]
    selected_indices = np.flatnonzero(component_inliers)
    component_inliers[selected_indices[~refined]] = False
    if len(inlier_points) < int(settings.get("minimum_component_points", 150)):
        raise RuntimeError("Too few tabletop points remain after component plane refinement")

    base_x = np.array([1.0, 0.0, 0.0])
    base_x -= normal * float(np.dot(base_x, normal))
    if np.linalg.norm(base_x) < 1e-6:
        base_x = np.array([0.0, 1.0, 0.0])
        base_x -= normal * float(np.dot(base_x, normal))
    base_x /= np.linalg.norm(base_x)
    base_y = np.cross(normal, base_x)
    local_xy = np.column_stack(
        ((inlier_points - plane_origin) @ base_x, (inlier_points - plane_origin) @ base_y)
    )
    q_low, q_high = [float(value) for value in settings["extent_quantiles"]]
    if not 0.0 <= q_low < q_high <= 1.0:
        raise ValueError("table_detection.extent_quantiles must satisfy 0 <= low < high <= 1")
    axis_x_2d, axis_y_2d, bounds, rectangle_point_count = fit_oriented_rectangle(
        local_xy, (q_low, q_high)
    )
    basis_x = base_x * axis_x_2d[0] + base_y * axis_x_2d[1]
    basis_y = np.cross(normal, basis_x)
    basis_y /= np.linalg.norm(basis_y)
    basis_x = np.cross(basis_y, normal)
    basis_x /= np.linalg.norm(basis_x)
    rectangle_local_center = (bounds[0] + bounds[1]) / 2.0
    center = (
        plane_origin
        + rectangle_local_center[0] * basis_x
        + rectangle_local_center[1] * basis_y
    )
    center -= normal * (float(np.dot(normal, center)) + plane_d)
    table_size = bounds[1] - bounds[0]
    half_size_xy = table_size / 2.0
    corners_local = np.array(
        [
            [-half_size_xy[0], -half_size_xy[1]],
            [half_size_xy[0], -half_size_xy[1]],
            [half_size_xy[0], half_size_xy[1]],
            [-half_size_xy[0], half_size_xy[1]],
        ]
    )
    corners_mujoco = center + corners_local[:, :1] * basis_x + corners_local[:, 1:] * basis_y
    short_side, long_side = sorted(float(value) for value in table_size)
    short_range = tuple(float(value) for value in settings.get("short_side_range_m", [0.20, 1.50]))
    long_range = tuple(float(value) for value in settings.get("long_side_range_m", [0.30, 2.50]))
    if not short_range[0] <= short_side <= short_range[1]:
        raise RuntimeError(
            f"Fitted tabletop short side {short_side:.3f} m is outside {short_range}"
        )
    if not long_range[0] <= long_side <= long_range[1]:
        raise RuntimeError(
            f"Fitted tabletop long side {long_side:.3f} m is outside {long_range}"
        )
    rotation = np.column_stack((basis_x, basis_y, normal))
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = center
    tilt_deg = float(np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0))))
    uphill_yaw_deg = float(np.degrees(np.arctan2(-normal[1], -normal[0])))
    quaternion = rotation_matrix_to_quaternion_wxyz(rotation)
    collision_thickness = float(settings["collision_thickness_m"])
    collision_center = center - normal * collision_thickness / 2.0
    collision_half_size = np.array([table_size[0] / 2.0, table_size[1] / 2.0, collision_thickness / 2.0])
    pos_text = " ".join(f"{value:.6f}" for value in collision_center)
    quat_text = " ".join(f"{value:.6f}" for value in quaternion)
    size_text = " ".join(f"{value:.6f}" for value in collision_half_size)

    table = {
        "plane_equation": {
            "form": "normal dot [x,y,z] + d = 0",
            "normal_mujoco": as_list(normal),
            "d_m": round(plane_d, 6),
        },
        "center_mujoco_m": as_list(center),
        "height_at_mujoco_xy_origin_m": round(float(-plane_d / normal[2]), 6) if abs(normal[2]) > 1e-8 else None,
        "orientation": {
            "plane_frame_quaternion_wxyz": as_list(quaternion),
            "plane_frame_rpy_deg": as_list(rotation_matrix_to_rpy_degrees(rotation), 4),
            "tilt_from_horizontal_deg": round(tilt_deg, 4),
            "uphill_direction_yaw_mujoco_deg": round(uphill_yaw_deg, 4),
        },
        "visible_extent": {
            "basis_x_mujoco": as_list(basis_x),
            "basis_y_mujoco": as_list(basis_y),
            "local_xy_bounds_m": [as_list(-half_size_xy), as_list(half_size_xy)],
            "size_xy_m": as_list(table_size),
            "corners_mujoco_m": [[round(float(value), 6) for value in row] for row in corners_mujoco],
            "method": "3d_ransac_connected_component_oriented_rectangle",
            "note": "Robust rectangular bounds of the connected visible tabletop patch.",
        },
        "mujoco_collision_box": {
            "type": "box",
            "pos_mujoco_m": as_list(collision_center),
            "quat_wxyz": as_list(quaternion),
            "size_half_extents_m": as_list(collision_half_size),
            "thickness_m": collision_thickness,
            "xml": f'<geom name="table_collision" type="box" pos="{pos_text}" quat="{quat_text}" size="{size_text}"/>',
            "note": "Top surface, center, orientation and XY size come from the same fitted 3D rectangle.",
        },
        "fit_quality": {
            "roi_uv_fraction": settings["roi_uv_fraction"],
            "sample_count": int(len(points_mujoco)),
            "inlier_count": inlier_count,
            "inlier_ratio": round(inlier_ratio, 6),
            "distance_threshold_m": threshold,
            "median_residual_m": round(float(np.median(residuals_inlier)), 6),
            "p95_residual_m": round(float(np.percentile(residuals_inlier, 95)), 6),
            "max_tilt_from_horizontal_deg": max_tilt_deg,
            "height_range_mujoco_m": list(height_range),
            "rectangle_fit_point_count": rectangle_point_count,
            **component_report,
        },
        "T_mujoco_table_center": transform.tolist(),
    }
    return table, pixels_uv[component_inliers], corners_mujoco


def project_mujoco_points_to_image(
    points_mujoco: np.ndarray,
    matrix: np.ndarray,
    point_offset: np.ndarray,
    camera: dict[str, Any],
) -> np.ndarray:
    inverse = np.linalg.inv(matrix)
    points_camera = (inverse @ np.column_stack((points_mujoco - point_offset, np.ones(len(points_mujoco)))).T).T[:, :3]
    intrinsic = camera["color"]["intrinsics"]
    distortion = camera["color"]["distortion"]
    camera_matrix = np.array(
        [[intrinsic["fx"], 0.0, intrinsic["cx"]], [0.0, intrinsic["fy"], intrinsic["cy"]], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    coefficients = np.array(
        [distortion["k1"], distortion["k2"], distortion["p1"], distortion["p2"], distortion["k3"],
         distortion["k4"], distortion["k5"], distortion["k6"]], dtype=np.float64
    )
    projected, _ = cv2.projectPoints(points_camera, np.zeros(3), np.zeros(3), camera_matrix, coefficients)
    return projected.reshape(-1, 2)


def main() -> None:
    args = parse_args()
    frame_dir = args.frame_dir.resolve()
    output_dir = (args.output_dir or frame_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    clear_previous_output(output_dir)

    color_path = frame_dir / "color.png"
    depth_path = frame_dir / "depth.png"
    metadata_path = frame_dir / "camera.json"
    for path in (color_path, depth_path, metadata_path, args.weights):
        if not path.exists():
            raise FileNotFoundError(path)

    color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    camera = read_json(metadata_path)
    if color is None or depth_raw is None:
        raise RuntimeError("Failed to load color.png or depth.png")
    if depth_raw.dtype != np.uint16:
        raise TypeError(f"Expected uint16 depth PNG, got {depth_raw.dtype}")
    if color.shape[:2] != depth_raw.shape:
        raise ValueError(f"Color/depth shape mismatch: {color.shape[:2]} vs {depth_raw.shape}")

    if args.device == "auto":
        device = "0" if torch.cuda.is_available() else "cpu"
    else:
        device = args.device
    # Ultralytics 8.4.90 YOLOE segmentation has a Half/Float mismatch in
    # retina-mask postprocessing, so keep FP32 on both CPU and CUDA.
    use_half = False

    os.chdir(ROOT)
    model = YOLO(str(args.weights.resolve()))
    model.set_classes(args.classes)
    predict_args: dict[str, Any] = {
        "source": str(color_path),
        "imgsz": args.imgsz,
        "conf": args.conf,
        "device": device,
        "retina_masks": True,
        "verbose": True,
    }
    result = model.predict(**predict_args)[0]

    detections: list[dict[str, Any]] = []
    masks: list[np.ndarray] = []
    if result.boxes is not None:
        mask_tensor = result.masks.data.cpu().numpy() if result.masks is not None else None
        for index, box in enumerate(result.boxes.cpu()):
            class_id = int(box.cls.item())
            item = {
                "index": index,
                "class_id": class_id,
                "class_name": result.names[class_id],
                "confidence": round(float(box.conf.item()), 6),
                "xyxy": [round(float(value), 2) for value in box.xyxy[0].tolist()],
            }
            if mask_tensor is not None:
                mask = mask_tensor[index]
                if mask.shape != depth_raw.shape:
                    mask = cv2.resize(mask, (depth_raw.shape[1], depth_raw.shape[0]), interpolation=cv2.INTER_NEAREST)
                mask_u8 = (mask > 0.5).astype(np.uint8) * 255
                item["mask_pixel_count"] = int(np.count_nonzero(mask_u8))
                masks.append(mask_u8)
            detections.append(item)

    selected_index = select_detection(detections, args)
    if not masks or selected_index >= len(masks):
        raise RuntimeError("Selected YOLOE detection has no segmentation mask")
    selected = detections[selected_index]
    selected_mask = masks[selected_index]
    selected_mask_path = output_dir / "selected_mask.png"
    cv2.imwrite(str(selected_mask_path), selected_mask)
    selected["mask_path"] = str(selected_mask_path)

    binary_mask = selected_mask > 0
    if args.mask_erode_px > 0:
        kernel_size = args.mask_erode_px * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        inner_mask = cv2.erode(selected_mask, kernel) > 0
        if np.count_nonzero(inner_mask) < 30:
            inner_mask = binary_mask
    else:
        inner_mask = binary_mask

    scale_m = float(camera["depth"]["scale_m_per_unit"])
    depth_m = depth_raw.astype(np.float64) * scale_m
    inner_pixel_count = int(np.count_nonzero(inner_mask))
    valid = inner_mask & (depth_raw > 0) & (depth_m >= args.min_depth_m) & (depth_m <= args.max_depth_m)
    raw_valid_pixel_count = int(np.count_nonzero(valid))
    rows, cols = np.nonzero(valid)
    if len(rows) < 30:
        raise RuntimeError(f"Only {len(rows)} valid depth pixels inside selected mask")

    depths = depth_m[rows, cols]
    median_depth = float(np.median(depths))
    mad = float(np.median(np.abs(depths - median_depth)))
    depth_p05, depth_p95 = [float(value) for value in np.percentile(depths, [5, 95])]
    depth_spread = depth_p95 - depth_p05
    valid_depth_fraction = raw_valid_pixel_count / max(inner_pixel_count, 1)
    quality_warnings: list[str] = []
    if valid_depth_fraction < 0.75:
        quality_warnings.append(
            "Less than 75% of the eroded mask has valid depth; transparent/reflective material or occlusion is likely."
        )
    if depth_spread > 0.08:
        quality_warnings.append(
            "The 5th-to-95th percentile depth spread exceeds 8 cm; the mask may contain multiple surfaces or see-through depth."
        )
    robust_sigma = 1.4826 * mad
    band = max(args.min_depth_band_m, args.mad_multiplier * robust_sigma)
    inlier = np.abs(depths - median_depth) <= band
    rows, cols, depths = rows[inlier], cols[inlier], depths[inlier]
    if len(rows) < 30:
        raise RuntimeError(f"Only {len(rows)} depth inliers remain after robust filtering")

    pixels_uv = np.column_stack((cols, rows)).astype(np.float64)
    intrinsics = camera["color"]["intrinsics"]
    distortion = camera["color"]["distortion"]
    rays, deprojection_method = pixels_to_rays(pixels_uv, intrinsics, distortion)
    points_camera = rays * depths[:, None]

    mask_moments = cv2.moments(selected_mask, binaryImage=True)
    mask_center_uv = np.array(
        [mask_moments["m10"] / mask_moments["m00"], mask_moments["m01"] / mask_moments["m00"]], dtype=np.float64
    )
    center_index = int(np.argmin(np.sum((pixels_uv - mask_center_uv) ** 2, axis=1)))
    surface_uv = pixels_uv[center_index]
    surface_point = points_camera[center_index]
    cloud_centroid = np.median(points_camera, axis=0)
    normal_camera, normal_point_count, normal_eigenvalues = estimate_surface_normal(
        points_camera, pixels_uv, surface_uv, args.normal_window_px, surface_point
    )
    pregrasp_camera = surface_point + args.pregrasp_offset_m * normal_camera

    calibration_path = args.calibration.resolve()
    calibration = read_json(calibration_path)
    matrix = np.asarray(calibration["T_mujoco_camera_nominal"], dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError("T_mujoco_camera_nominal must be 4x4")
    configured_offset = np.asarray(calibration["point_offset_mujoco_m"], dtype=np.float64)
    if configured_offset.shape != (3,):
        raise ValueError("point_offset_mujoco_m must contain exactly three values")
    point_offset = (
        np.asarray(args.offset_mujoco_m, dtype=np.float64)
        if args.offset_mujoco_m is not None
        else configured_offset
    )
    surface_mujoco = transform_point(matrix, surface_point) + point_offset
    centroid_mujoco = transform_point(matrix, cloud_centroid) + point_offset
    normal_mujoco = transform_vector(matrix, normal_camera)
    pregrasp_mujoco = transform_point(matrix, pregrasp_camera) + point_offset
    excluded_table_mask = np.zeros(depth_raw.shape, dtype=bool)
    exclusion_dilate_px = int(
        calibration["table_detection"].get("object_exclusion_dilate_px", 12)
    )
    exclusion_kernel = None
    if exclusion_dilate_px > 0:
        exclusion_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * exclusion_dilate_px + 1, 2 * exclusion_dilate_px + 1),
        )
    for instance_mask in masks:
        exclusion = (
            cv2.dilate(instance_mask, exclusion_kernel)
            if exclusion_kernel is not None
            else instance_mask
        )
        excluded_table_mask |= exclusion > 0
    table_result, table_inlier_uv, table_corners_mujoco = estimate_table_plane(
        depth_raw,
        camera,
        excluded_table_mask,
        matrix,
        point_offset,
        calibration["table_detection"],
        anchor_uv=surface_uv,
    )
    mujoco_result = {
        "coordinate_frame": calibration["coordinate_system"]["name"],
        "axes": calibration["coordinate_system"]["axes"],
        "reference_pose": calibration["reference_pose"],
        "calibration_source": str(calibration_path),
        "T_mujoco_camera_nominal": matrix.tolist(),
        "point_offset_mujoco_m_applied": as_list(point_offset),
        "point_offset_source": "command_line" if args.offset_mujoco_m is not None else "calibration_file",
        "surface_point_mujoco_m": as_list(surface_mujoco),
        "visible_cloud_centroid_mujoco_m": as_list(centroid_mujoco),
        "local_pca_normal_mujoco_experimental": as_list(normal_mujoco),
        "pregrasp_point_mujoco_m_experimental": as_list(pregrasp_mujoco),
        "warning": calibration["warning"],
    }

    output = {
        "schema_version": 1,
        "grasp_point_mujoco_m": as_list(surface_mujoco),
        "table_plane_mujoco": table_result,
        "source": {
            "frame_dir": str(frame_dir),
            "color": str(color_path),
            "depth": str(depth_path),
            "camera_metadata": str(metadata_path),
            "camera_serial_number": camera["device"]["serial_number"],
        },
        "inference": {
            "weights": str(args.weights.resolve()),
            "device": device,
            "fp16": use_half,
            "classes": args.classes,
            "confidence_threshold": args.conf,
            "speed_ms": result.speed,
        },
        "detections": detections,
        "selected_detection_index": selected_index,
        "selected_detection": selected,
        "depth_filter": {
            "erode_px": args.mask_erode_px,
            "valid_range_m": [args.min_depth_m, args.max_depth_m],
            "inner_mask_pixel_count": inner_pixel_count,
            "raw_valid_pixel_count": raw_valid_pixel_count,
            "valid_depth_fraction": round(valid_depth_fraction, 6),
            "inlier_pixel_count": int(len(points_camera)),
            "median_depth_m": round(median_depth, 6),
            "mad_m": round(mad, 6),
            "depth_p05_m": round(depth_p05, 6),
            "depth_p95_m": round(depth_p95, 6),
            "depth_p05_to_p95_spread_m": round(depth_spread, 6),
            "accepted_half_band_m": round(band, 6),
            "quality_warnings": quality_warnings,
        },
        "deprojection": {
            "method": deprojection_method,
            "coordinate_frame": "aligned color optical frame (+x right, +y down, +z forward)",
            "mask_center_uv": as_list(mask_center_uv, 2),
            "surface_pixel_uv": as_list(surface_uv, 2),
            "surface_point_camera_m": as_list(surface_point),
            "visible_cloud_centroid_camera_m": as_list(cloud_centroid),
            "local_pca_normal_camera_experimental": as_list(normal_camera),
            "pregrasp_offset_m": args.pregrasp_offset_m,
            "pregrasp_point_camera_m_experimental": as_list(pregrasp_camera),
            "normal_fit_point_count": normal_point_count,
            "normal_fit_eigenvalues": as_list(normal_eigenvalues, 10),
        },
        "mujoco": mujoco_result,
        "status": "visual_debug_3d_point_only_not_an_executable_6d_grasp_pose",
    }
    output_path = output_dir / "result.json"
    output_path.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")

    annotated = color.copy()
    table_pixels = np.rint(table_inlier_uv).astype(int)
    table_mask = np.zeros(depth_raw.shape, dtype=np.uint8)
    table_mask[table_pixels[:, 1], table_pixels[:, 0]] = 255
    table_mask = cv2.dilate(table_mask, np.ones((5, 5), dtype=np.uint8)) > 0
    cyan = np.zeros_like(annotated)
    cyan[:, :, 0] = 255
    cyan[:, :, 1] = 180
    table_overlay = cv2.addWeighted(annotated, 0.72, cyan, 0.28, 0.0)
    annotated[table_mask] = table_overlay[table_mask]

    projected_corners = np.rint(
        project_mujoco_points_to_image(table_corners_mujoco, matrix, point_offset, camera)
    ).astype(int)
    cv2.polylines(annotated, [projected_corners.reshape(-1, 1, 2)], True, (255, 255, 0), 3)

    green = np.zeros_like(annotated)
    green[:, :, 1] = 255
    masked_overlay = cv2.addWeighted(annotated, 0.65, green, 0.35, 0.0)
    annotated[binary_mask] = masked_overlay[binary_mask]
    x1, y1, x2, y2 = [int(round(value)) for value in selected["xyxy"]]
    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 220, 255), 2)
    center_xy = tuple(np.rint(surface_uv).astype(int))
    cv2.drawMarker(annotated, center_xy, (0, 0, 255), cv2.MARKER_CROSS, 28, 3)
    label = f"{selected['class_name']} {selected['confidence']:.2f} z={surface_point[2]:.3f}m"
    cv2.putText(annotated, label, (max(5, x1), max(25, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)
    xyz_text = f"cam xyz=({surface_point[0]:+.3f}, {surface_point[1]:+.3f}, {surface_point[2]:+.3f})m"
    cv2.putText(annotated, xyz_text, (20, color.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    table_size = table_result["visible_extent"]["size_xy_m"]
    table_text = (
        f"table tilt={table_result['orientation']['tilt_from_horizontal_deg']:.1f}deg "
        f"visible={table_size[0]:.2f}x{table_size[1]:.2f}m"
    )
    cv2.putText(annotated, table_text, (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 0), 2)
    cv2.imwrite(str(output_dir / "annotated.png"), annotated)

    print(f"Selected: #{selected_index} {selected['class_name']} conf={selected['confidence']:.3f}")
    print(f"Surface camera xyz [m]: {as_list(surface_point)}")
    print(f"Experimental local PCA normal: {as_list(normal_camera)}")
    if quality_warnings:
        print("Depth quality warnings:")
        for warning in quality_warnings:
            print(f"  - {warning}")
    print(f"MuJoCo world xyz [m]: {as_list(surface_mujoco)}")
    print(f"MuJoCo point offset [m]: {as_list(point_offset)}")
    print(f"Table center MuJoCo [m]: {table_result['center_mujoco_m']}")
    print(f"Table normal MuJoCo: {table_result['plane_equation']['normal_mujoco']}")
    print(f"Table tilt [deg]: {table_result['orientation']['tilt_from_horizontal_deg']}")
    print(f"Table visible size [m]: {table_result['visible_extent']['size_xy_m']}")
    print(f"Result: {output_path}")


if __name__ == "__main__":
    main()
