"""Target-selection safety tests for the YOLOE RGB-D pipeline."""

from argparse import Namespace

import numpy as np
import pytest

pytest.importorskip("cv2")
pytest.importorskip("torch")
pytest.importorskip("ultralytics")

from vision.yoloe_depth_target import (  # noqa: E402
    DEFAULT_CLASSES,
    evaluate_detection_candidate,
    select_detection,
)


def _args(**overrides: object) -> Namespace:
    values = {
        "detection_index": None,
        "target_class": None,
        "mask_erode_px": 0,
        "min_depth_m": 0.15,
        "max_depth_m": 2.0,
        "min_depth_band_m": 0.012,
        "mad_multiplier": 3.5,
    }
    values.update(overrides)
    return Namespace(**values)


def _candidate_fixture(
    *,
    mask_bounds: tuple[int, int, int, int] = (45, 45, 55, 55),
    depth_units: int = 900,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict, dict, np.ndarray]:
    mask = np.zeros((100, 100), dtype=np.uint8)
    x1, y1, x2, y2 = mask_bounds
    mask[y1:y2, x1:x2] = 255
    depth_raw = np.zeros((100, 100), dtype=np.uint16)
    depth_raw[mask > 0] = depth_units
    depth_m = depth_raw.astype(np.float64) * 0.001
    camera = {
        "color": {
            "intrinsics": {"fx": 100.0, "fy": 100.0, "cx": 50.0, "cy": 50.0},
            "distortion": {"model": "none"},
        }
    }
    table_result = {
        "plane_equation": {"normal_mujoco": [0.0, 0.0, 1.0], "d_m": -0.8},
        "visible_extent": {"size_xy_m": [2.0, 2.0]},
        "T_mujoco_table_center": [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.8],
            [0.0, 0.0, 0.0, 1.0],
        ],
    }
    projected_corners = np.array(
        [[20.0, 20.0], [80.0, 20.0], [80.0, 80.0], [20.0, 80.0]],
        dtype=np.float64,
    )
    return mask, depth_raw, depth_m, camera, table_result, projected_corners


def _evaluate(
    *,
    mask_bounds: tuple[int, int, int, int] = (45, 45, 55, 55),
    depth_units: int = 900,
    table_size_xy_m: tuple[float, float] = (2.0, 2.0),
    settings_overrides: dict[str, float] | None = None,
) -> tuple[dict, dict | None]:
    mask, depth_raw, depth_m, camera, table_result, corners = _candidate_fixture(
        mask_bounds=mask_bounds,
        depth_units=depth_units,
    )
    table_result["visible_extent"]["size_xy_m"] = list(table_size_xy_m)
    settings = {
        "minimum_height_above_table_m": 0.01,
        "table_polygon_margin_px": 0.0,
        "table_footprint_margin_m": 0.0,
        "table_footprint_tolerance_m": 0.0,
    }
    settings.update(settings_overrides or {})
    return evaluate_detection_candidate(
        mask,
        depth_raw,
        depth_m,
        camera,
        _args(),
        np.eye(4),
        np.zeros(3),
        table_result,
        corners,
        settings,
    )


def test_default_competition_classes_are_specific() -> None:
    assert DEFAULT_CLASSES == [
        "cup",
        "orange-capped pill bottle",
        "bag of corn bread",
    ]


def test_selection_ignores_a_higher_confidence_rejected_detection() -> None:
    detections = [
        {
            "index": 0,
            "class_name": "cup",
            "confidence": 0.95,
            "candidate_filter": {
                "accepted": False,
                "rejection_reasons": ["mask_center_outside_table_image_region"],
            },
        },
        {
            "index": 1,
            "class_name": "cup",
            "confidence": 0.80,
            "candidate_filter": {"accepted": True, "rejection_reasons": []},
        },
    ]
    assert select_detection(detections, _args(target_class="cup")) == 1


def test_explicit_detection_index_cannot_bypass_candidate_filter() -> None:
    detections = [
        {
            "index": 0,
            "class_name": "cup",
            "confidence": 0.95,
            "candidate_filter": {
                "accepted": False,
                "rejection_reasons": ["surface_not_high_enough_above_table"],
            },
        }
    ]
    with pytest.raises(RuntimeError, match="outside the safe tabletop"):
        select_detection(detections, _args(detection_index=0))


def test_candidate_accepts_centered_surface_above_table() -> None:
    report, depth_analysis = _evaluate()
    assert report["accepted"] is True
    assert report["mask_center_inside_table_image_region"] is True
    assert report["surface_projection_inside_table_footprint"] is True
    assert report["height_above_table_m"] == pytest.approx(0.1)
    assert depth_analysis is not None


def test_candidate_rejects_mask_center_outside_table_polygon() -> None:
    report, _ = _evaluate(mask_bounds=(85, 45, 95, 55))
    assert report["accepted"] is False
    assert "mask_center_outside_table_image_region" in report["rejection_reasons"]


def test_candidate_rejects_surface_too_close_to_table_plane() -> None:
    report, _ = _evaluate(depth_units=805)
    assert report["accepted"] is False
    assert "surface_not_high_enough_above_table" in report["rejection_reasons"]


def test_candidate_allows_small_table_edge_projection_tolerance() -> None:
    rejected, _ = _evaluate(
        mask_bounds=(55, 45, 65, 55),
        table_size_xy_m=(0.1, 2.0),
    )
    accepted, _ = _evaluate(
        mask_bounds=(55, 45, 65, 55),
        table_size_xy_m=(0.1, 2.0),
        settings_overrides={"table_footprint_tolerance_m": 0.05},
    )

    assert rejected["accepted"] is False
    assert "surface_projection_outside_table_footprint" in rejected[
        "rejection_reasons"
    ]
    assert accepted["accepted"] is True
    assert accepted["surface_projection_overflow_xy_m"][0] > 0.0
