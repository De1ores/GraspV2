"""Dependency-light geometry shared by vision and grasp planning contracts."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


DEFAULT_SIDE_GRASP_HEIGHT_OFFSET_M = 0.01
MINIMUM_CENTER_ESTIMATION_POINTS = 30


@dataclass(frozen=True)
class ObjectCenterEstimate:
    """Table-relative object and gripper centers estimated from one RGB-D mask."""

    object_center_world_m: tuple[float, float, float]
    gripper_center_world_m: tuple[float, float, float]
    object_height_m: float
    visible_height_percentiles_m: tuple[float, float]
    method: str = "mask_centroid_ray_at_robust_visible_height_midpoint"


def _vector(value: Iterable[float], label: str) -> np.ndarray:
    result = np.asarray(tuple(value), dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain three finite values")
    return result


def estimate_object_and_gripper_centers(
    points_world_m: np.ndarray,
    camera_origin_world_m: Iterable[float],
    mask_centroid_ray_world: Iterable[float],
    table_normal_world: Iterable[float],
    table_plane_d_m: float,
    *,
    gripper_height_offset_m: float = DEFAULT_SIDE_GRASP_HEIGHT_OFFSET_M,
) -> ObjectCenterEstimate:
    """Estimate centers without relabeling a visible surface sample as a target.

    Robust lower/upper heights of the segmented cloud are measured relative to
    the table, and their midpoint defines the observed object center height.
    The mask-centroid camera ray is intersected with that plane. This corrects
    camera-facing surface bias and remains valid after the object leaves the
    table during lift verification.
    """

    points = np.asarray(points_world_m, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (3,):
        raise ValueError("points_world_m must have shape (N, 3)")
    if len(points) < MINIMUM_CENTER_ESTIMATION_POINTS:
        raise ValueError(
            "too few world points for object-center estimation: "
            f"{len(points)} < {MINIMUM_CENTER_ESTIMATION_POINTS}"
        )
    if not np.all(np.isfinite(points)):
        raise ValueError("points_world_m contains non-finite values")

    origin = _vector(camera_origin_world_m, "camera_origin_world_m")
    ray = _vector(mask_centroid_ray_world, "mask_centroid_ray_world")
    normal = _vector(table_normal_world, "table_normal_world")
    ray_norm = float(np.linalg.norm(ray))
    normal_norm = float(np.linalg.norm(normal))
    if ray_norm <= 1e-12 or normal_norm <= 1e-12:
        raise ValueError("camera ray and table normal must be non-zero")
    ray /= ray_norm
    normal /= normal_norm
    plane_d = float(table_plane_d_m) / normal_norm
    offset = float(gripper_height_offset_m)
    if not math.isfinite(plane_d) or not math.isfinite(offset):
        raise ValueError("table plane and gripper offset must be finite")

    heights = points @ normal + plane_d
    positive = heights[np.isfinite(heights) & (heights > 0.0)]
    if len(positive) < MINIMUM_CENTER_ESTIMATION_POINTS:
        raise ValueError(
            "too few segmented points lie above the fitted table for "
            "object-center estimation"
        )
    low, high = (float(value) for value in np.percentile(positive, [5.0, 95.0]))
    object_height = high - low
    if not math.isfinite(object_height) or object_height <= 1e-6:
        raise ValueError(
            "segmented cloud has no measurable table-normal height extent"
        )
    center_height = (low + high) / 2.0

    denominator = float(np.dot(normal, ray))
    if abs(denominator) <= 1e-9:
        raise ValueError("mask-centroid ray is parallel to the table plane")
    origin_height = float(np.dot(normal, origin) + plane_d)
    distance = (center_height - origin_height) / denominator
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError("object-center plane lies behind the camera")

    object_center = origin + distance * ray
    # Remove floating-point residue so the reported center is exactly at the
    # documented table-relative height.
    center_error = float(np.dot(normal, object_center) + plane_d - center_height)
    object_center -= center_error * normal
    gripper_center = object_center + offset * normal
    return ObjectCenterEstimate(
        object_center_world_m=tuple(float(value) for value in object_center),
        gripper_center_world_m=tuple(float(value) for value in gripper_center),
        object_height_m=object_height,
        visible_height_percentiles_m=(low, high),
    )
