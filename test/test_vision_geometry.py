"""Pure tests for the visual object/gripper center contract."""

import numpy as np
import pytest

from graspv2.vision_geometry import estimate_object_and_gripper_centers


def test_center_estimate_intersects_mask_ray_at_half_object_height() -> None:
    heights = np.linspace(0.0, 0.20, 101)[1:]
    points = np.column_stack(
        (
            np.full_like(heights, 0.10),
            np.zeros_like(heights),
            heights,
        )
    )
    estimate = estimate_object_and_gripper_centers(
        points,
        camera_origin_world_m=(0.0, 0.0, 1.0),
        mask_centroid_ray_world=(0.2, 0.0, -1.0),
        table_normal_world=(0.0, 0.0, 1.0),
        table_plane_d_m=0.0,
        gripper_height_offset_m=0.005,
    )

    expected_low, expected_high = np.percentile(heights, [5.0, 95.0])
    expected_height = expected_high - expected_low
    expected_center_height = (expected_low + expected_high) / 2.0
    assert estimate.object_height_m == pytest.approx(expected_height)
    assert estimate.object_center_world_m[2] == pytest.approx(
        expected_center_height
    )
    assert estimate.gripper_center_world_m[2] == pytest.approx(
        expected_center_height + 0.005
    )
    # The center follows the silhouette-centroid ray, rather than copying the
    # camera-facing surface cloud's constant X coordinate.
    assert estimate.object_center_world_m[0] != pytest.approx(0.10)


def test_center_estimate_rejects_too_few_points() -> None:
    with pytest.raises(ValueError, match="too few world points"):
        estimate_object_and_gripper_centers(
            np.zeros((10, 3)),
            camera_origin_world_m=(0.0, 0.0, 1.0),
            mask_centroid_ray_world=(0.0, 0.0, -1.0),
            table_normal_world=(0.0, 0.0, 1.0),
            table_plane_d_m=0.0,
        )


def test_center_estimate_tracks_full_lift_displacement() -> None:
    heights = np.linspace(0.01, 0.19, 100)
    points = np.column_stack(
        (np.zeros_like(heights), np.zeros_like(heights), heights)
    )
    lifted_points = points + np.array([0.0, 0.0, 0.045])
    arguments = {
        "camera_origin_world_m": (0.0, 0.0, 1.0),
        "mask_centroid_ray_world": (0.0, 0.0, -1.0),
        "table_normal_world": (0.0, 0.0, 1.0),
        "table_plane_d_m": 0.0,
    }

    initial = estimate_object_and_gripper_centers(points, **arguments)
    lifted = estimate_object_and_gripper_centers(lifted_points, **arguments)
    assert (
        lifted.object_center_world_m[2] - initial.object_center_world_m[2]
    ) == pytest.approx(0.045)
