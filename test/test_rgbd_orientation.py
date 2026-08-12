"""Tests for synchronized stored RGB-D orientation changes."""

import json
import os
import subprocess
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from vision.rgbd_orientation import (  # noqa: E402
    calibrated_rotation_deg,
    resolve_camera_transform,
    rotate_rgbd_frame_180,
    stored_rotation_deg,
)


ROOT = Path(__file__).resolve().parents[1]


def _camera(rotation_deg: int | float | None = 0) -> dict:
    document = {
        "color": {
            "intrinsics": {
                "fx": 100.0,
                "fy": 101.0,
                "cx": 1.25,
                "cy": 0.75,
                "width": 4,
                "height": 2,
            },
            "distortion": {"model": "brown_conrady", "p1": 0.01, "p2": -0.02},
        },
        "depth": {
            "source_intrinsics": {
                "fx": 90.0,
                "fy": 91.0,
                "cx": 1.5,
                "cy": 0.5,
                "width": 4,
                "height": 2,
            },
            "source_distortion": {"model": "none", "p1": 0.0, "p2": 0.0},
        },
    }
    if rotation_deg is not None:
        document["image_orientation"] = {"rotation_deg": rotation_deg}
    return document


def test_legacy_frame_defaults_to_zero_degrees() -> None:
    assert stored_rotation_deg(_camera(None)) == 0


def test_fractional_rotation_is_rejected() -> None:
    with pytest.raises(ValueError, match="must be 0 or 180"):
        stored_rotation_deg(_camera(180.5))


@pytest.mark.parametrize(
    ("mount_orientation", "rotation_deg"),
    (("upright", 0), ("inverted", 180)),
)
def test_camera_mount_profile_selects_matching_stored_rotation(
    mount_orientation: str, rotation_deg: int
) -> None:
    calibration = {
        "model_values": {
            "camera_mount_orientation": mount_orientation,
            "capture_image_rotation_deg": rotation_deg,
        }
    }
    assert calibrated_rotation_deg(calibration) == rotation_deg


def test_camera_mount_profile_rejects_orientation_mismatch() -> None:
    calibration = {
        "model_values": {
            "camera_mount_orientation": "upright",
            "capture_image_rotation_deg": 180,
        }
    }
    with pytest.raises(ValueError, match="camera mount/profile mismatch"):
        calibrated_rotation_deg(calibration)


def test_half_turn_updates_rgb_depth_intrinsics_and_distortion() -> None:
    color = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    depth = np.arange(8, dtype=np.uint16).reshape(2, 4)
    rotated_color, rotated_depth, camera = rotate_rgbd_frame_180(
        color, depth, _camera(0)
    )

    np.testing.assert_array_equal(rotated_color, color[::-1, ::-1])
    np.testing.assert_array_equal(rotated_depth, depth[::-1, ::-1])
    assert stored_rotation_deg(camera) == 180
    assert camera["color"]["intrinsics"]["cx"] == pytest.approx(1.75)
    assert camera["color"]["intrinsics"]["cy"] == pytest.approx(0.25)
    assert camera["color"]["distortion"]["p1"] == pytest.approx(-0.01)
    assert camera["color"]["distortion"]["p2"] == pytest.approx(0.02)
    assert camera["depth"]["source_intrinsics"]["cx"] == pytest.approx(1.5)


def test_two_half_turns_restore_frame_and_calibration() -> None:
    color = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    depth = np.arange(8, dtype=np.uint16).reshape(2, 4)
    first = rotate_rgbd_frame_180(color, depth, _camera(0))
    second = rotate_rgbd_frame_180(*first)

    np.testing.assert_array_equal(second[0], color)
    np.testing.assert_array_equal(second[1], depth)
    assert stored_rotation_deg(second[2]) == 0
    assert second[2]["color"]["intrinsics"]["cx"] == pytest.approx(1.25)
    assert second[2]["color"]["distortion"]["p1"] == pytest.approx(0.01)


def test_camera_transform_compensates_for_alternate_stored_rotation() -> None:
    nominal = np.array(
        [
            [0.0, -1.0, 0.0, 0.1],
            [1.0, 0.0, 0.0, 0.2],
            [0.0, 0.0, 1.0, 0.3],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    calibration = {
        "T_mujoco_camera_nominal": nominal.tolist(),
        "model_values": {
            "camera_mount_orientation": "inverted",
            "capture_image_rotation_deg": 180,
        },
    }

    _, effective, report = resolve_camera_transform(
        calibration, {"image_orientation": {"rotation_deg": 0}}
    )

    expected = nominal @ np.diag([-1.0, -1.0, 1.0, 1.0])
    np.testing.assert_allclose(effective, expected)
    assert report == {
        "calibration_rotation_deg": 180,
        "stored_rotation_deg": 0,
        "relative_transform_rotation_deg": 180,
        "adjusted": True,
        "camera_mount_orientation": "inverted",
    }


def test_camera_transform_is_nominal_at_calibrated_rotation() -> None:
    nominal = np.eye(4)
    calibration = {
        "T_mujoco_camera_nominal": nominal.tolist(),
        "model_values": {
            "camera_mount_orientation": "inverted",
            "capture_image_rotation_deg": 180,
        },
    }

    _, effective, report = resolve_camera_transform(
        calibration, {"image_orientation": {"rotation_deg": 180}}
    )

    np.testing.assert_allclose(effective, nominal)
    assert report["adjusted"] is False
    assert report["camera_mount_orientation"] == "inverted"


@pytest.mark.parametrize("stored_rotation", (0, 180))
def test_camera_transform_auto_treats_each_rotation_as_mount_hypothesis(
    stored_rotation: int,
) -> None:
    nominal = np.array(
        [
            [0.0, -0.6, 0.8, 0.1],
            [-1.0, 0.0, 0.0, 0.2],
            [0.0, -0.8, -0.6, 1.1],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    calibration = {
        "T_mujoco_camera_nominal": nominal.tolist(),
        "model_values": {
            "camera_mount_orientation": "inverted",
            "capture_image_rotation_deg": 180,
        },
    }
    camera = _camera(stored_rotation)
    camera["image_orientation"]["selection_mode"] = "auto"

    _, effective, report = resolve_camera_transform(calibration, camera)

    np.testing.assert_allclose(effective, nominal)
    assert report["calibration_rotation_deg"] == 180
    assert report["stored_rotation_deg"] == stored_rotation
    assert report["relative_transform_rotation_deg"] == 0
    assert report["adjusted"] is False
    assert report["auto_mount_hypothesis"] is True
    assert report["transform_reference_rotation_deg"] == stored_rotation


def _write_frame(frame_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    frame_dir.mkdir()
    color = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    depth = np.arange(8, dtype=np.uint16).reshape(2, 4)
    assert cv2.imwrite(str(frame_dir / "color.png"), color)
    assert cv2.imwrite(str(frame_dir / "depth.png"), depth)
    (frame_dir / "camera.json").write_text(
        json.dumps(_camera(0)), encoding="utf-8"
    )
    return color, depth


def _write_fake_vision_python(
    path: Path,
    accepted_rotation: int | None,
    *,
    failure_status: int = 7,
) -> None:
    accepted = "None" if accepted_rotation is None else str(accepted_rotation)
    path.write_text(
        "#!/usr/bin/python3\n"
        "import json\n"
        "import sys\n"
        f"accepted = {accepted}\n"
        "frame_dir = sys.argv[sys.argv.index('--frame-dir') + 1]\n"
        "with open(frame_dir + '/camera.json', encoding='utf-8') as stream:\n"
        "    rotation = json.load(stream)['image_orientation']['rotation_deg']\n"
        f"raise SystemExit(0 if rotation == accepted else {failure_status})\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def _write_mount_calibration(path: Path, mount_orientation: str) -> None:
    rotation_deg = {"upright": 0, "inverted": 180}[mount_orientation]
    path.write_text(
        json.dumps(
            {
                "model_values": {
                    "camera_mount_orientation": mount_orientation,
                    "capture_image_rotation_deg": rotation_deg,
                }
            }
        ),
        encoding="utf-8",
    )


def _write_fake_ros_capture_python(path: Path, status: int) -> None:
    path.write_text(
        f"#!/usr/bin/env bash\nexit {status}\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def _write_fake_orbbec_capture(path: Path) -> None:
    path.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "output=\n"
        "while (($#)); do\n"
        "  case \"$1\" in\n"
        "    --output) output=\"$2\"; shift 2 ;;\n"
        "    --warmup|--timeout) shift 2 ;;\n"
        "    *) exit 2 ;;\n"
        "  esac\n"
        "done\n"
        "mkdir -p \"$output\"\n"
        "cp \"$FAKE_RGBD_SOURCE/color.png\" \"$output/color.png\"\n"
        "cp \"$FAKE_RGBD_SOURCE/depth.png\" \"$output/depth.png\"\n"
        "cp \"$FAKE_RGBD_SOURCE/camera.json\" \"$output/camera.json\"\n"
        "printf sdk > \"$FAKE_SDK_MARKER\"\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def test_run_vision_auto_falls_back_to_sdk_only_on_topic_timeout(
    tmp_path: Path,
) -> None:
    source_dir = tmp_path / "sdk-frame"
    _write_frame(source_dir)
    output_dir = tmp_path / "output"
    calibration = tmp_path / "upright-calibration.json"
    _write_mount_calibration(calibration, "upright")
    fake_ros_python = tmp_path / "fake-ros-python"
    _write_fake_ros_capture_python(fake_ros_python, status=20)
    fake_sdk = tmp_path / "fake-orbbec-capture"
    _write_fake_orbbec_capture(fake_sdk)
    marker = tmp_path / "sdk-used"
    environment = {
        **os.environ,
        "GRASPV2_X2_ENV_READY": "1",
        "GRASPV2_ROS_CAPTURE_PYTHON": str(fake_ros_python),
        "FAKE_RGBD_SOURCE": str(source_dir),
        "FAKE_SDK_MARKER": str(marker),
    }

    completed = subprocess.run(
        [
            str(ROOT / "run_vision.sh"),
            "--capture-backend",
            "auto",
            "--capture-only",
            "--output-dir",
            str(output_dir),
            "--calibration",
            str(calibration),
            "--orbbec-binary",
            str(fake_sdk),
        ],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert "falling back to the local Orbbec SDK" in completed.stderr
    assert "RGB-D source selected: local Orbbec SDK" in completed.stdout
    assert marker.read_text(encoding="utf-8") == "sdk"
    assert (output_dir / "camera.json").is_file()


def test_run_vision_auto_does_not_hide_non_timeout_ros_failure(
    tmp_path: Path,
) -> None:
    source_dir = tmp_path / "sdk-frame"
    _write_frame(source_dir)
    output_dir = tmp_path / "output"
    calibration = tmp_path / "upright-calibration.json"
    _write_mount_calibration(calibration, "upright")
    fake_ros_python = tmp_path / "fake-ros-python"
    _write_fake_ros_capture_python(fake_ros_python, status=7)
    fake_sdk = tmp_path / "fake-orbbec-capture"
    _write_fake_orbbec_capture(fake_sdk)
    marker = tmp_path / "sdk-used"
    environment = {
        **os.environ,
        "GRASPV2_X2_ENV_READY": "1",
        "GRASPV2_ROS_CAPTURE_PYTHON": str(fake_ros_python),
        "FAKE_RGBD_SOURCE": str(source_dir),
        "FAKE_SDK_MARKER": str(marker),
    }

    completed = subprocess.run(
        [
            str(ROOT / "run_vision.sh"),
            "--capture-backend",
            "auto",
            "--capture-only",
            "--output-dir",
            str(output_dir),
            "--calibration",
            str(calibration),
            "--orbbec-binary",
            str(fake_sdk),
        ],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 7
    assert "SDK fallback is limited to topic timeout" in completed.stderr
    assert not marker.exists()


def test_run_vision_auto_retries_180_only_after_table_failure(
    tmp_path: Path,
) -> None:
    frame_dir = tmp_path / "frame"
    original_color, original_depth = _write_frame(frame_dir)
    fake_python = tmp_path / "fake-vision-python"
    _write_fake_vision_python(
        fake_python, accepted_rotation=180, failure_status=42
    )
    calibration = tmp_path / "upright-calibration.json"
    _write_mount_calibration(calibration, "upright")
    environment = {**os.environ, "GRASPV2_VISION_PYTHON": str(fake_python)}
    environment.pop("GRASPV2_RGBD_ROTATION_DEG", None)

    completed = subprocess.run(
        [
            str(ROOT / "run_vision.sh"),
            "--capture-backend",
            "existing",
            "--output-dir",
            str(frame_dir),
            "--calibration",
            str(calibration),
            "--image-rotation-deg",
            "auto",
        ],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    camera = json.loads((frame_dir / "camera.json").read_text(encoding="utf-8"))
    assert camera["image_orientation"]["rotation_deg"] == 180
    assert camera["image_orientation"]["selection_mode"] == "auto"
    assert camera["image_orientation"]["attempt"] == 2
    np.testing.assert_array_equal(
        cv2.imread(str(frame_dir / "color.png")), original_color[::-1, ::-1]
    )
    np.testing.assert_array_equal(
        cv2.imread(str(frame_dir / "depth.png"), cv2.IMREAD_UNCHANGED),
        original_depth[::-1, ::-1],
    )


def test_run_vision_auto_restores_frame_when_both_attempts_fail(
    tmp_path: Path,
) -> None:
    frame_dir = tmp_path / "frame"
    original_color, original_depth = _write_frame(frame_dir)
    fake_python = tmp_path / "fake-vision-python"
    _write_fake_vision_python(
        fake_python, accepted_rotation=None, failure_status=42
    )
    calibration = tmp_path / "upright-calibration.json"
    _write_mount_calibration(calibration, "upright")
    environment = {**os.environ, "GRASPV2_VISION_PYTHON": str(fake_python)}

    completed = subprocess.run(
        [
            str(ROOT / "run_vision.sh"),
            "--capture-backend",
            "existing",
            "--output-dir",
            str(frame_dir),
            "--image-rotation-deg",
            "auto",
            "--calibration",
            str(calibration),
        ],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 42
    camera = json.loads((frame_dir / "camera.json").read_text(encoding="utf-8"))
    assert camera["image_orientation"]["rotation_deg"] == 0
    assert camera["image_orientation"]["attempt"] == 1
    np.testing.assert_array_equal(
        cv2.imread(str(frame_dir / "color.png")), original_color
    )
    np.testing.assert_array_equal(
        cv2.imread(str(frame_dir / "depth.png"), cv2.IMREAD_UNCHANGED),
        original_depth,
    )


def test_run_vision_auto_does_not_rotate_for_non_table_failure(
    tmp_path: Path,
) -> None:
    frame_dir = tmp_path / "frame"
    original_color, original_depth = _write_frame(frame_dir)
    fake_python = tmp_path / "fake-vision-python"
    _write_fake_vision_python(
        fake_python, accepted_rotation=180, failure_status=7
    )
    calibration = tmp_path / "inverted-calibration.json"
    _write_mount_calibration(calibration, "inverted")
    environment = {**os.environ, "GRASPV2_VISION_PYTHON": str(fake_python)}

    completed = subprocess.run(
        [
            str(ROOT / "run_vision.sh"),
            "--capture-backend",
            "existing",
            "--output-dir",
            str(frame_dir),
            "--calibration",
            str(calibration),
            "--image-rotation-deg",
            "auto",
        ],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 7
    assert "keeping RGB-D at 0 degrees" in completed.stderr
    camera = json.loads((frame_dir / "camera.json").read_text(encoding="utf-8"))
    assert camera["image_orientation"]["rotation_deg"] == 0
    assert camera["image_orientation"]["attempt"] == 1
    np.testing.assert_array_equal(
        cv2.imread(str(frame_dir / "color.png")), original_color
    )
    np.testing.assert_array_equal(
        cv2.imread(str(frame_dir / "depth.png"), cv2.IMREAD_UNCHANGED),
        original_depth,
    )


def test_run_vision_defaults_to_unrotated_table_aware_mode(
    tmp_path: Path,
) -> None:
    frame_dir = tmp_path / "frame"
    original_color, original_depth = _write_frame(frame_dir)
    fake_python = tmp_path / "fake-vision-python"
    _write_fake_vision_python(fake_python, accepted_rotation=0)
    calibration = tmp_path / "inverted-calibration.json"
    _write_mount_calibration(calibration, "inverted")
    environment = {**os.environ, "GRASPV2_VISION_PYTHON": str(fake_python)}
    environment.pop("GRASPV2_RGBD_ROTATION_DEG", None)

    completed = subprocess.run(
        [
            str(ROOT / "run_vision.sh"),
            "--capture-backend",
            "existing",
            "--output-dir",
            str(frame_dir),
            "--calibration",
            str(calibration),
        ],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    camera = json.loads((frame_dir / "camera.json").read_text(encoding="utf-8"))
    assert camera["image_orientation"]["rotation_deg"] == 0
    assert camera["image_orientation"]["selection_mode"] == "auto"
    assert camera["image_orientation"]["attempt"] == 1
    np.testing.assert_array_equal(
        cv2.imread(str(frame_dir / "color.png")), original_color
    )
    np.testing.assert_array_equal(
        cv2.imread(str(frame_dir / "depth.png"), cv2.IMREAD_UNCHANGED),
        original_depth,
    )


def test_run_vision_explicit_calibrated_uses_mount_orientation(
    tmp_path: Path,
) -> None:
    frame_dir = tmp_path / "frame"
    original_color, original_depth = _write_frame(frame_dir)
    calibration = tmp_path / "inverted-calibration.json"
    _write_mount_calibration(calibration, "inverted")

    completed = subprocess.run(
        [
            str(ROOT / "run_vision.sh"),
            "--capture-backend",
            "existing",
            "--capture-only",
            "--output-dir",
            str(frame_dir),
            "--calibration",
            str(calibration),
            "--image-rotation-deg",
            "calibrated",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    camera = json.loads((frame_dir / "camera.json").read_text(encoding="utf-8"))
    assert camera["image_orientation"]["rotation_deg"] == 180
    assert camera["image_orientation"]["selection_mode"] == "calibrated"
    np.testing.assert_array_equal(
        cv2.imread(str(frame_dir / "color.png")), original_color[::-1, ::-1]
    )
    np.testing.assert_array_equal(
        cv2.imread(str(frame_dir / "depth.png"), cv2.IMREAD_UNCHANGED),
        original_depth[::-1, ::-1],
    )
