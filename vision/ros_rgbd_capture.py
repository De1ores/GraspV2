#!/usr/bin/env python3
"""Capture one synchronized, color-aligned RGB-D pair from X2 AimDK topics."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from graspv2.hardware_contract import (  # noqa: E402
    DEFAULT_HARDWARE_CONFIG_PATH,
    load_hardware_config,
)
from graspv2.ros_logging import configure_fastdds_logging  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument(
        "--hardware-config",
        type=Path,
        default=DEFAULT_HARDWARE_CONFIG_PATH,
        help="graspV2 X2 AimDK topic/tuning JSON",
    )
    selected, _ = config_parser.parse_known_args(argv)
    hardware = load_hardware_config(selected.hardware_config)

    parser = argparse.ArgumentParser(description=__doc__, parents=[config_parser])
    parser.add_argument("--color-topic", default=hardware.topics.rgb_image)
    parser.add_argument("--depth-topic", default=hardware.topics.depth_image)
    parser.add_argument(
        "--camera-info-topic",
        default=hardware.topics.rgb_camera_info,
        help="RGB CameraInfo topic used by the detector",
    )
    parser.add_argument(
        "--depth-camera-info-topic",
        default=hardware.topics.depth_camera_info,
    )
    parser.add_argument("--output-dir", type=Path, default=ROOT / "output")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--max-skew-ms", type=float, default=50.0)
    parser.add_argument(
        "--depth-scale-m",
        type=float,
        default=0.001,
        help="Meters represented by one 16UC1 depth unit (ROS convention: 0.001)",
    )
    parser.add_argument(
        "--image-rotation-deg",
        type=int,
        choices=(0, 180),
        default=180,
        help=(
            "Rotate both registered RGB and depth before storage. The X2 "
            "competition camera is mounted upside down, so the default is 180."
        ),
    )
    return parser.parse_args(argv)


def _stamp_ns(message: Image) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _rows(message: Image, dtype: np.dtype[Any], channels: int) -> np.ndarray:
    item_size = dtype.itemsize
    if message.step % item_size:
        raise ValueError(
            f"Image step {message.step} is not divisible by item size {item_size}"
        )
    row_items = message.step // item_size
    visible_items = int(message.width) * channels
    if row_items < visible_items:
        raise ValueError(
            f"Image step exposes {row_items} items, expected at least {visible_items}"
        )
    values = np.frombuffer(message.data, dtype=dtype)
    required = int(message.height) * row_items
    if values.size < required:
        raise ValueError(
            f"Image data has {values.size} items, expected at least {required}"
        )
    return values[:required].reshape(int(message.height), row_items)[:, :visible_items]


def color_to_bgr(message: Image) -> np.ndarray:
    encoding = message.encoding.lower()
    if encoding in {"rgb8", "bgr8"}:
        image = _rows(message, np.dtype(np.uint8), 3).reshape(
            message.height, message.width, 3
        )
        return (
            cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if encoding == "rgb8"
            else image.copy()
        )
    if encoding in {"rgba8", "bgra8"}:
        image = _rows(message, np.dtype(np.uint8), 4).reshape(
            message.height, message.width, 4
        )
        conversion = (
            cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR
        )
        return cv2.cvtColor(image, conversion)
    if encoding in {"mono8", "8uc1"}:
        image = _rows(message, np.dtype(np.uint8), 1).reshape(
            message.height, message.width
        )
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"Unsupported color encoding: {message.encoding}")


def depth_to_u16(message: Image, depth_scale_m: float) -> np.ndarray:
    encoding = message.encoding.lower()
    endian = ">" if message.is_bigendian else "<"
    if encoding in {"16uc1", "mono16"}:
        image = _rows(message, np.dtype(endian + "u2"), 1).reshape(
            message.height, message.width
        )
        return image.astype(np.uint16, copy=True)
    if encoding == "32fc1":
        depth_m = _rows(message, np.dtype(endian + "f4"), 1).reshape(
            message.height, message.width
        )
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        image = np.zeros(depth_m.shape, dtype=np.uint16)
        scaled = np.rint(depth_m[valid] / depth_scale_m)
        image[valid] = np.clip(
            scaled, 0, np.iinfo(np.uint16).max
        ).astype(np.uint16)
        return image
    raise ValueError(f"Unsupported depth encoding: {message.encoding}")


@dataclass(frozen=True)
class CapturedPair:
    color: Image
    depth: Image
    color_camera_info: CameraInfo
    depth_camera_info: CameraInfo
    skew_ms: float


class RgbdCapture(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("graspv2_x2_rgbd_capture")
        self.args = args
        self.latest_color: Image | None = None
        self.latest_depth: Image | None = None
        self.color_camera_info: CameraInfo | None = None
        self.depth_camera_info: CameraInfo | None = None
        self.result: CapturedPair | None = None
        self.matched_frames = 0
        self.last_pair_stamps: tuple[int, int] | None = None
        self.create_subscription(
            Image,
            args.color_topic,
            self._color_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            args.depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            self._color_camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            args.depth_camera_info_topic,
            self._depth_camera_info_callback,
            qos_profile_sensor_data,
        )

    def _color_callback(self, message: Image) -> None:
        self.latest_color = message
        self._try_match()

    def _depth_callback(self, message: Image) -> None:
        self.latest_depth = message
        self._try_match()

    def _color_camera_info_callback(self, message: CameraInfo) -> None:
        self.color_camera_info = message
        self._try_match()

    def _depth_camera_info_callback(self, message: CameraInfo) -> None:
        self.depth_camera_info = message
        self._try_match()

    def _try_match(self) -> None:
        if self.result is not None or not all(
            (
                self.latest_color,
                self.latest_depth,
                self.color_camera_info,
                self.depth_camera_info,
            )
        ):
            return
        assert self.latest_color is not None
        assert self.latest_depth is not None
        assert self.color_camera_info is not None
        assert self.depth_camera_info is not None
        color_stamp = _stamp_ns(self.latest_color)
        depth_stamp = _stamp_ns(self.latest_depth)
        stamps = (color_stamp, depth_stamp)
        if stamps == self.last_pair_stamps:
            return
        skew_ms = abs(color_stamp - depth_stamp) / 1_000_000.0
        if skew_ms > self.args.max_skew_ms:
            return
        self.last_pair_stamps = stamps
        self.matched_frames += 1
        if self.matched_frames >= self.args.warmup_frames:
            self.result = CapturedPair(
                self.latest_color,
                self.latest_depth,
                self.color_camera_info,
                self.depth_camera_info,
                skew_ms,
            )


def _distortion(
    camera_info: CameraInfo, image_rotation_deg: int = 0
) -> dict[str, float | str]:
    coefficients = list(camera_info.d) + [0.0] * max(0, 8 - len(camera_info.d))
    if image_rotation_deg == 180:
        # Brown-Conrady tangential coefficients change sign when both normalized
        # image axes are negated. Radial coefficients remain unchanged.
        coefficients[2] = -coefficients[2]
        coefficients[3] = -coefficients[3]
    return {
        "model": camera_info.distortion_model or "none",
        "k1": float(coefficients[0]),
        "k2": float(coefficients[1]),
        "p1": float(coefficients[2]),
        "p2": float(coefficients[3]),
        "k3": float(coefficients[4]),
        "k4": float(coefficients[5]),
        "k5": float(coefficients[6]),
        "k6": float(coefficients[7]),
    }


def _intrinsics(
    camera_info: CameraInfo, image_rotation_deg: int = 0
) -> dict[str, float | int]:
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    width = int(camera_info.width)
    height = int(camera_info.height)
    if image_rotation_deg == 180:
        cx = (width - 1) - cx
        cy = (height - 1) - cy
    return {
        "fx": float(camera_info.k[0]),
        "fy": float(camera_info.k[4]),
        "cx": cx,
        "cy": cy,
        "width": width,
        "height": height,
    }


def rotate_rgbd_180(
    color: np.ndarray, depth: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Rotate an aligned RGB-D pair without introducing interpolation."""
    if color.shape[:2] != depth.shape:
        raise ValueError(
            f"Color/depth shape mismatch before rotation: {color.shape[:2]} vs {depth.shape}"
        )
    return cv2.rotate(color, cv2.ROTATE_180), cv2.rotate(depth, cv2.ROTATE_180)


def _camera_matrix(camera_info: CameraInfo) -> np.ndarray:
    if len(camera_info.k) != 9:
        raise ValueError("CameraInfo.k must contain 9 values")
    matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("CameraInfo does not contain valid pinhole intrinsics")
    return matrix


def _distortion_coefficients(camera_info: CameraInfo) -> np.ndarray:
    values = list(camera_info.d)
    values.extend([0.0] * max(0, 8 - len(values)))
    return np.asarray(values[:8], dtype=np.float64)


def _undistort_pixels(
    pixels_uv: np.ndarray, camera_info: CameraInfo
) -> np.ndarray:
    matrix = _camera_matrix(camera_info)
    coefficients = _distortion_coefficients(camera_info)
    model = (camera_info.distortion_model or "none").lower()
    points = pixels_uv.astype(np.float64).reshape(-1, 1, 2)
    if model in {"equidistant", "kannala_brandt4"}:
        normalized = cv2.fisheye.undistortPoints(
            points, matrix, coefficients[:4]
        ).reshape(-1, 2)
    elif model != "none" and np.any(np.abs(coefficients) > 1e-12):
        normalized = cv2.undistortPoints(
            points, matrix, coefficients
        ).reshape(-1, 2)
    else:
        normalized = np.column_stack(
            (
                (pixels_uv[:, 0] - matrix[0, 2]) / matrix[0, 0],
                (pixels_uv[:, 1] - matrix[1, 2]) / matrix[1, 1],
            )
        )
    return np.column_stack((normalized, np.ones(len(normalized))))


def _project_points(
    points_camera: np.ndarray, camera_info: CameraInfo
) -> np.ndarray:
    matrix = _camera_matrix(camera_info)
    coefficients = _distortion_coefficients(camera_info)
    model = (camera_info.distortion_model or "none").lower()
    if model in {"equidistant", "kannala_brandt4"}:
        pixels, _ = cv2.fisheye.projectPoints(
            points_camera.reshape(-1, 1, 3),
            np.zeros(3),
            np.zeros(3),
            matrix,
            coefficients[:4],
        )
    else:
        active_coefficients = (
            coefficients
            if model != "none" and np.any(np.abs(coefficients) > 1e-12)
            else np.zeros(5, dtype=np.float64)
        )
        pixels, _ = cv2.projectPoints(
            points_camera,
            np.zeros(3),
            np.zeros(3),
            matrix,
            active_coefficients,
        )
    return pixels.reshape(-1, 2)


def align_depth_to_color(
    depth: np.ndarray,
    depth_scale_m: float,
    depth_camera_info: CameraInfo,
    color_camera_info: CameraInfo,
    *,
    depth_frame_id: str,
    color_frame_id: str,
) -> tuple[np.ndarray, str, int]:
    """Reproject X2 depth pixels into the RGB pixel grid.

    The X2 SDK documents both streams in ``rgbd_head_front``. If firmware
    reports different frames, an extrinsic is required and capture is rejected
    instead of silently combining unrelated pixels.
    """

    if depth.ndim != 2 or depth.dtype != np.uint16:
        raise ValueError("depth must be a uint16 image")
    if depth_scale_m <= 0.0:
        raise ValueError("depth_scale_m must be positive")
    if not depth_frame_id or not color_frame_id:
        raise ValueError("RGB and depth images must provide frame_id")
    if depth_frame_id != color_frame_id:
        raise ValueError(
            "X2 RGB/depth frame_id mismatch; depth-to-color extrinsics are "
            f"required ({depth_frame_id!r} -> {color_frame_id!r})"
        )
    depth_height, depth_width = depth.shape
    if (int(depth_camera_info.width), int(depth_camera_info.height)) != (
        depth_width,
        depth_height,
    ):
        raise ValueError(
            "Depth CameraInfo dimensions do not match depth image: "
            f"info={depth_camera_info.width}x{depth_camera_info.height}, "
            f"image={depth_width}x{depth_height}"
        )
    color_width = int(color_camera_info.width)
    color_height = int(color_camera_info.height)
    if color_width <= 0 or color_height <= 0:
        raise ValueError("RGB CameraInfo dimensions must be positive")

    same_geometry = (
        (depth_width, depth_height) == (color_width, color_height)
        and np.allclose(
            _camera_matrix(depth_camera_info),
            _camera_matrix(color_camera_info),
            atol=1e-9,
            rtol=0.0,
        )
        and (depth_camera_info.distortion_model or "none")
        == (color_camera_info.distortion_model or "none")
        and np.allclose(
            _distortion_coefficients(depth_camera_info),
            _distortion_coefficients(color_camera_info),
            atol=1e-12,
            rtol=0.0,
        )
    )
    valid_v, valid_u = np.nonzero(depth)
    if len(valid_u) == 0:
        raise ValueError("depth image contains no valid samples")
    if same_geometry:
        return depth.copy(), "identity_same_intrinsics", int(len(valid_u))

    pixels_depth = np.column_stack((valid_u, valid_v)).astype(np.float64)
    rays = _undistort_pixels(pixels_depth, depth_camera_info)
    depths_m = depth[valid_v, valid_u].astype(np.float64) * depth_scale_m
    points = rays * depths_m[:, None]
    pixels_color = np.rint(_project_points(points, color_camera_info)).astype(
        np.int64
    )
    inside = (
        (pixels_color[:, 0] >= 0)
        & (pixels_color[:, 0] < color_width)
        & (pixels_color[:, 1] >= 0)
        & (pixels_color[:, 1] < color_height)
    )
    if not np.any(inside):
        raise ValueError("depth reprojection produced no pixels inside the RGB image")
    target = pixels_color[inside]
    source_depth = depth[valid_v[inside], valid_u[inside]].astype(np.uint32)
    target_index = target[:, 1] * color_width + target[:, 0]
    sentinel = np.iinfo(np.uint32).max
    z_buffer = np.full(color_width * color_height, sentinel, dtype=np.uint32)
    np.minimum.at(z_buffer, target_index, source_depth)
    populated = z_buffer != sentinel
    aligned = np.zeros(color_width * color_height, dtype=np.uint16)
    aligned[populated] = z_buffer[populated].astype(np.uint16)
    return (
        aligned.reshape(color_height, color_width),
        "same_frame_intrinsics_reprojection",
        int(np.count_nonzero(populated)),
    )


def write_capture(pair: CapturedPair, args: argparse.Namespace) -> None:
    color = color_to_bgr(pair.color)
    raw_depth = depth_to_u16(pair.depth, args.depth_scale_m)
    color_info = pair.color_camera_info
    depth_info = pair.depth_camera_info
    for label, image, camera_info in (
        ("RGB", pair.color, color_info),
        ("Depth", pair.depth, depth_info),
    ):
        image_frame = str(image.header.frame_id)
        info_frame = str(camera_info.header.frame_id)
        if not info_frame:
            raise ValueError(f"{label} CameraInfo must provide frame_id")
        if info_frame != image_frame:
            raise ValueError(
                f"{label} CameraInfo frame_id {info_frame!r} does not match "
                f"image frame_id {image_frame!r}"
            )
    if (int(color_info.width), int(color_info.height)) != (
        color.shape[1],
        color.shape[0],
    ):
        raise ValueError(
            "RGB CameraInfo dimensions do not match RGB image: "
            f"info={color_info.width}x{color_info.height}, "
            f"image={color.shape[1]}x{color.shape[0]}"
        )
    depth, registration, registered_samples = align_depth_to_color(
        raw_depth,
        args.depth_scale_m,
        depth_info,
        color_info,
        depth_frame_id=pair.depth.header.frame_id,
        color_frame_id=pair.color.header.frame_id,
    )
    image_rotation_deg = int(args.image_rotation_deg)
    if image_rotation_deg == 180:
        color, depth = rotate_rgbd_180(color, depth)
        registration = f"{registration}_then_rotate_180"

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    color_path = output_dir / "color.png"
    depth_path = output_dir / "depth.png"
    if not cv2.imwrite(str(color_path), color):
        raise RuntimeError(f"Failed to write {color_path}")
    if not cv2.imwrite(
        str(depth_path), depth, [cv2.IMWRITE_PNG_COMPRESSION, 0]
    ):
        raise RuntimeError(f"Failed to write {depth_path}")

    metadata = {
        "schema_version": 1,
        "device": {
            "name": "X2 AimDK head-front RGB-D camera",
            "serial_number": "topic-source",
            "connection_type": "ros2_aimdk",
        },
        "files": {"color": "color.png", "depth": "depth.png"},
        "alignment": "depth_to_color",
        "depth_registration": {
            "method": registration,
            "registered_sample_count": registered_samples,
            "frame_id": pair.color.header.frame_id,
        },
        "image_orientation": {
            "rotation_deg": image_rotation_deg,
            "reason": (
                "X2 competition RGB-D camera is physically mounted upside down"
                if image_rotation_deg == 180
                else "rotation disabled by capture option"
            ),
            "applied_to": ["color.png", "depth.png", "camera intrinsics"],
        },
        "camera_coordinate_convention": (
            "stored upright color optical frame: +x right, +y down, +z forward"
        ),
        "ros_topics": {
            "color": args.color_topic,
            "depth": args.depth_topic,
            "rgb_camera_info": args.camera_info_topic,
            "depth_camera_info": args.depth_camera_info_topic,
        },
        "color": {
            "width": int(pair.color.width),
            "height": int(pair.color.height),
            "timestamp_ms": _stamp_ns(pair.color) / 1_000_000.0,
            "frame_id": pair.color.header.frame_id,
            "encoding": pair.color.encoding,
            "intrinsics": _intrinsics(color_info, image_rotation_deg),
            "distortion": _distortion(color_info, image_rotation_deg),
        },
        "depth": {
            "width": int(depth.shape[1]),
            "height": int(depth.shape[0]),
            "source_width": int(pair.depth.width),
            "source_height": int(pair.depth.height),
            "timestamp_ms": _stamp_ns(pair.depth) / 1_000_000.0,
            "frame_id": pair.depth.header.frame_id,
            "encoding": pair.depth.encoding,
            "storage": "uint16_png",
            "value_scale_mm_per_unit": args.depth_scale_m * 1000.0,
            "scale_m_per_unit": args.depth_scale_m,
            "source_intrinsics": _intrinsics(depth_info, image_rotation_deg),
            "source_distortion": _distortion(depth_info, image_rotation_deg),
        },
        "synchronization": {"skew_ms": pair.skew_ms},
    }
    metadata_path = output_dir / "camera.json"
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(
        f"Captured X2 RGB-D pair: {color.shape[1]}x{color.shape[0]}, "
        f"skew={pair.skew_ms:.3f} ms"
    )
    print(f"Color topic: {args.color_topic} ({pair.color.encoding})")
    print(
        f"Depth topic: {args.depth_topic} ({pair.depth.encoding}, "
        f"scale={args.depth_scale_m} m/unit)"
    )
    print(f"Depth registration: {registration}, samples={registered_samples}")
    print(f"Stored RGB-D rotation: {image_rotation_deg} deg")
    print(f"Output: {output_dir}")


def main() -> int:
    args = parse_args()
    if args.timeout <= 0.0 or args.max_skew_ms < 0.0 or args.depth_scale_m <= 0.0:
        raise ValueError(
            "timeout and depth scale must be positive; max skew must be non-negative"
        )
    if args.warmup_frames < 1:
        raise ValueError("warmup frames must be at least 1")

    configure_fastdds_logging()
    rclpy.init(args=[])
    node = RgbdCapture(args)
    deadline = time.monotonic() + args.timeout
    try:
        while node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result is None:
            raise TimeoutError(
                "Timed out waiting for synchronized RGB-D data on "
                f"{args.color_topic}, {args.depth_topic}, "
                f"{args.camera_info_topic}, and {args.depth_camera_info_topic}"
            )
        write_capture(node.result, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
