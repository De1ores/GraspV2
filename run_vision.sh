#!/usr/bin/env bash
set -euo pipefail

# Keep ROS system Python isolated from stale packages in ~/.local.  X2 images
# are decoded by the OpenCV/NumPy pair installed with Ubuntu/AimDK.
export PYTHONNOUSERSITE=1

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "$repo_dir/.vision-venv/bin/python" ]]; then
  default_vision_python="$repo_dir/.vision-venv/bin/python"
else
  default_vision_python="$repo_dir/.venv/bin/python"
fi
vision_python="${GRASPV2_VISION_PYTHON:-$default_vision_python}"
if [[ -d "$repo_dir/.runtime/cusparselt/lib" ]]; then
  export LD_LIBRARY_PATH="$repo_dir/.runtime/cusparselt/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
cupti_lib="$repo_dir/.vision-venv/lib/python3.10/site-packages/nvidia/cuda_cupti/lib"
if [[ -d "$cupti_lib" ]]; then
  export LD_LIBRARY_PATH="$cupti_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
capture_backend="existing"
capture_only=false
output_dir="$repo_dir/output"
orbbec_binary="${GRASPV2_ORBBEC_CAPTURE:-$repo_dir/build/orbbec_capture}"
capture_timeout="20"
warmup_frames="10"
depth_scale_m="0.001"
image_rotation_deg="${GRASPV2_RGBD_ROTATION_DEG:-180}"
hardware_config="$repo_dir/config/x2_aimdk_hardware.json"
color_topic=""
depth_topic=""
rgb_info_topic=""
depth_info_topic=""
vision_args=()

usage() {
  cat <<'EOF'
Usage: ./run_vision.sh --capture-backend x2-aimdk|orbbec-sdk|existing [options] [YOLOE options]

Capture backends:
  x2-aimdk    Subscribe to the official X2 RGB/depth Image and CameraInfo topics.
  orbbec-sdk  Invoke the existing manual Orbbec SDK capture binary.
  existing    Do not capture; consume output/color.png, depth.png and camera.json.

Capture options:
  --capture-only                 Stop after writing the RGB-D frame contract.
  --output-dir DIR               Frame directory (default: output).
  --hardware-config PATH         AimDK topic/tuning config.
  --color-topic TOPIC            Override X2 RGB Image topic.
  --depth-topic TOPIC            Override X2 depth Image topic.
  --rgb-camera-info-topic TOPIC  Override X2 RGB CameraInfo topic.
  --depth-camera-info-topic TOPIC
  --capture-timeout SEC          X2 topic timeout (default: 20).
  --warmup-frames N              X2 matched-frame warmup (default: 10).
  --depth-scale-m VALUE          X2 16UC1 metres/unit (default: 0.001).
  --image-rotation-deg 0|180     Rotate aligned RGB-D (default: 180 for upside-down X2 camera).
  --orbbec-binary PATH           Manual SDK capture binary.

All unrecognized options are forwarded to vision/yoloe_depth_target.py.
EOF
}

while (($#)); do
  case "$1" in
    --capture-backend|--output-dir|--hardware-config|--color-topic|--depth-topic|--rgb-camera-info-topic|--depth-camera-info-topic|--capture-timeout|--warmup-frames|--depth-scale-m|--image-rotation-deg|--orbbec-binary)
      (($# >= 2)) || { echo "$1 requires a value" >&2; exit 2; }
      case "$1" in
        --capture-backend) capture_backend="$2" ;;
        --output-dir) output_dir="$2" ;;
        --hardware-config) hardware_config="$2" ;;
        --color-topic) color_topic="$2" ;;
        --depth-topic) depth_topic="$2" ;;
        --rgb-camera-info-topic) rgb_info_topic="$2" ;;
        --depth-camera-info-topic) depth_info_topic="$2" ;;
        --capture-timeout) capture_timeout="$2" ;;
        --warmup-frames) warmup_frames="$2" ;;
        --depth-scale-m) depth_scale_m="$2" ;;
        --image-rotation-deg) image_rotation_deg="$2" ;;
        --orbbec-binary) orbbec_binary="$2" ;;
      esac
      shift 2
      ;;
    --capture-only)
      capture_only=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      vision_args+=("$1")
      shift
      ;;
  esac
done

case "$capture_backend" in
  x2-aimdk|orbbec-sdk|existing) ;;
  *) echo "--capture-backend must be x2-aimdk, orbbec-sdk, or existing" >&2; exit 2 ;;
esac

mkdir -p "$output_dir"
case "$capture_backend" in
  x2-aimdk)
    if [[ "${GRASPV2_X2_ENV_READY:-0}" != "1" ]]; then
      # Standalone X2 capture needs both ROS and the matching AimDK overlay.
      # shellcheck source=/dev/null
      source "$repo_dir/tools/setup_x2_mc_env.sh"
    fi
    capture_args=(
      --hardware-config "$hardware_config"
      --output-dir "$output_dir"
      --timeout "$capture_timeout"
      --warmup-frames "$warmup_frames"
      --depth-scale-m "$depth_scale_m"
      --image-rotation-deg "$image_rotation_deg"
    )
    [[ -z "$color_topic" ]] || capture_args+=(--color-topic "$color_topic")
    [[ -z "$depth_topic" ]] || capture_args+=(--depth-topic "$depth_topic")
    [[ -z "$rgb_info_topic" ]] || capture_args+=(--camera-info-topic "$rgb_info_topic")
    [[ -z "$depth_info_topic" ]] || capture_args+=(--depth-camera-info-topic "$depth_info_topic")
    /usr/bin/python3 "$repo_dir/vision/ros_rgbd_capture.py" "${capture_args[@]}"
    ;;
  orbbec-sdk)
    if [[ ! -x "$orbbec_binary" ]]; then
      echo "Orbbec SDK capture binary is missing: $orbbec_binary" >&2
      echo "Build it on the test machine with: ./tools/build_orbbec_capture.sh" >&2
      exit 1
    fi
    "$orbbec_binary" --output "$output_dir" --warmup "$warmup_frames"
    ;;
  existing)
    for required in color.png depth.png camera.json; do
      if [[ ! -f "$output_dir/$required" ]]; then
        echo "Existing RGB-D input is missing: $output_dir/$required" >&2
        exit 1
      fi
    done
    echo "Using manually captured RGB-D files from $output_dir"
    ;;
esac

if [[ "$capture_only" == true ]]; then
  exit 0
fi
if [[ ! -x "$vision_python" ]]; then
  echo "Vision Python environment is missing: $vision_python" >&2
  exit 1
fi

export GRASPV2_USE_BUNDLED_PYTHON=0
export YOLO_AUTOINSTALL=false
exec "$vision_python" "$repo_dir/vision/yoloe_depth_target.py" \
  --frame-dir "$output_dir" --output-dir "$output_dir" "${vision_args[@]}"
