"""RGB/depth registration tests for the X2 AimDK capture backend."""

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

pytest.importorskip("cv2")

from vision.ros_rgbd_capture import (  # noqa: E402
    _distortion,
    _intrinsics,
    align_depth_to_color,
    parse_args,
    rotate_rgbd_180,
)


def _camera_info(
    width: int,
    height: int,
    fx: float,
    fy: float,
    cx: float = 0.0,
    cy: float = 0.0,
) -> CameraInfo:
    result = CameraInfo()
    result.width = width
    result.height = height
    result.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    result.d = [0.0] * 5
    result.distortion_model = "none"
    return result


def test_capture_defaults_to_official_x2_topics() -> None:
    args = parse_args([])
    assert args.color_topic == "/aima/hal/sensor/rgbd_head_front/rgb_image"
    assert args.depth_topic == "/aima/hal/sensor/rgbd_head_front/depth_image"
    assert args.camera_info_topic.endswith("/rgb_camera_info")
    assert args.depth_camera_info_topic.endswith("/depth_camera_info")
    assert args.image_rotation_deg == 180


def test_upside_down_capture_rotates_rgb_depth_and_calibration() -> None:
    color = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    depth = np.array([[1, 2], [3, 4]], dtype=np.uint16)
    rotated_color, rotated_depth = rotate_rgbd_180(color, depth)
    np.testing.assert_array_equal(rotated_color, color[::-1, ::-1])
    np.testing.assert_array_equal(rotated_depth, depth[::-1, ::-1])

    camera = _camera_info(1280, 720, 688.0, 689.0, 641.5, 357.1)
    camera.distortion_model = "plumb_bob"
    camera.d = [0.01, -0.02, 0.003, -0.004, 0.005]
    intrinsics = _intrinsics(camera, 180)
    assert intrinsics["cx"] == pytest.approx(637.5)
    assert intrinsics["cy"] == pytest.approx(361.9)
    distortion = _distortion(camera, 180)
    assert distortion["k1"] == pytest.approx(0.01)
    assert distortion["p1"] == pytest.approx(-0.003)
    assert distortion["p2"] == pytest.approx(0.004)


def test_identity_registration_preserves_depth() -> None:
    camera = _camera_info(2, 2, 1.0, 1.0)
    depth = np.array([[1000, 0], [1500, 2000]], dtype=np.uint16)
    aligned, method, count = align_depth_to_color(
        depth,
        0.001,
        camera,
        camera,
        depth_frame_id="rgbd_head_front",
        color_frame_id="rgbd_head_front",
    )
    assert method == "identity_same_intrinsics"
    assert count == 3
    np.testing.assert_array_equal(aligned, depth)


def test_same_frame_reprojects_depth_with_separate_intrinsics() -> None:
    depth_info = _camera_info(2, 1, 1.0, 1.0)
    color_info = _camera_info(4, 1, 2.0, 1.0)
    depth = np.array([[1000, 2000]], dtype=np.uint16)
    aligned, method, count = align_depth_to_color(
        depth,
        0.001,
        depth_info,
        color_info,
        depth_frame_id="rgbd_head_front",
        color_frame_id="rgbd_head_front",
    )
    assert method == "same_frame_intrinsics_reprojection"
    assert count == 2
    assert aligned.tolist() == [[1000, 0, 2000, 0]]


def test_registration_rejects_unknown_extrinsics() -> None:
    camera = _camera_info(1, 1, 1.0, 1.0)
    with pytest.raises(ValueError, match="extrinsics are required"):
        align_depth_to_color(
            np.array([[1000]], dtype=np.uint16),
            0.001,
            camera,
            camera,
            depth_frame_id="depth",
            color_frame_id="rgb",
        )
