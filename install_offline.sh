#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
planning_wheels="$repo_dir/offline/wheelhouse/aarch64/planning"
vision_wheels="$repo_dir/offline/wheelhouse/aarch64/vision"
runtime_dir="$repo_dir/offline/runtime"
force=false

usage() {
  cat <<'EOF'
Usage: ./install_offline.sh [--force]

Create the two Python environments and build the graspv2 ROS package entirely
from this bundle. No package index or network download is used.
EOF
}

while (($#)); do
  case "$1" in
    --force) force=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ "$(uname -m)" == "aarch64" ]] || {
  echo "Offline bundle targets aarch64 Jetson, found $(uname -m)" >&2
  exit 1
}
[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "ROS 2 Humble is missing: /opt/ros/humble/setup.bash" >&2
  exit 1
}
[[ "$(/usr/bin/python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" == "3.10" ]] || {
  echo "The bundle requires system Python 3.10" >&2
  exit 1
}
[[ -d "$planning_wheels" && -d "$vision_wheels" ]] || {
  echo "Offline wheelhouse is incomplete; use the complete generated archive" >&2
  exit 1
}
[[ -f "$runtime_dir/libcusparse_lt-linux-aarch64-0.7.1.0-archive.tar.xz" ]] || {
  echo "Bundled cuSPARSELt archive is missing" >&2
  exit 1
}

if [[ -f "$repo_dir/offline/MANIFEST.sha256" ]]; then
  (cd "$repo_dir" && sha256sum --check --strict offline/MANIFEST.sha256)
fi

aimdk_setup="$("$repo_dir/tools/select_aimdk_setup.sh")"

prepare_venv() {
  local path="$1"
  if [[ -d "$path" && ! -x "$path/bin/python" ]]; then
    echo "Invalid environment exists at $path" >&2
    exit 1
  fi
  if [[ -d "$path" && "$force" == true ]]; then
    local backup="${path}.backup.$(date +%Y%m%d%H%M%S)"
    mv -- "$path" "$backup"
    echo "Moved old environment to $backup"
  fi
  if [[ ! -x "$path/bin/python" ]]; then
    /usr/bin/python3 -m venv "$path"
  fi
}

export PIP_NO_INDEX=1
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PYTHONNOUSERSITE=1
export YOLO_OFFLINE=true
export YOLO_AUTOINSTALL=false
export YOLO_CONFIG_DIR="$repo_dir/.runtime/ultralytics"

prepare_venv "$repo_dir/.planning-venv"
"$repo_dir/.planning-venv/bin/python" -m pip install \
  --no-index --find-links "$planning_wheels" \
  --requirement "$repo_dir/offline/requirements-planning.txt"

prepare_venv "$repo_dir/.vision-venv"
"$repo_dir/.vision-venv/bin/python" -m pip install \
  --no-index --find-links "$vision_wheels" \
  --requirement "$repo_dir/offline/requirements-vision.txt"
"$repo_dir/.vision-venv/bin/python" -m pip install --no-index --no-deps \
  "$vision_wheels/ultralytics-8.4.90-py3-none-any.whl" \
  "$vision_wheels/ultralytics_thop-2.0.18-py3-none-any.whl"

mkdir -p "$repo_dir/.runtime/cusparselt"
mkdir -p "$YOLO_CONFIG_DIR"
if [[ ! -e "$repo_dir/.runtime/cusparselt/lib/libcusparseLt.so.0" ]]; then
  tar --extract --xz \
    --file "$runtime_dir/libcusparse_lt-linux-aarch64-0.7.1.0-archive.tar.xz" \
    --directory "$repo_dir/.runtime/cusparselt" --strip-components=1
fi

export GRASPV2_AIMDK_SETUP="$aimdk_setup"
"$repo_dir/tools/build_graspv2_with_installed_aimdk.sh"
"$repo_dir/tools/build_orbbec_capture.sh" --if-available

cupti_lib="$repo_dir/.vision-venv/lib/python3.10/site-packages/nvidia/cuda_cupti/lib"
export LD_LIBRARY_PATH="$repo_dir/.runtime/cusparselt/lib:$cupti_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$repo_dir/.planning-venv/bin/python" - <<'PY'
import platform

import mujoco
import numpy
import pinocchio
import x2_ik_sdk

# This deliberately creates no renderer or OpenGL context.  It proves that the
# bundled ARM64 native library can compile and evaluate a model on the Orin
# before any vision capture or robot command is attempted.
model = mujoco.MjModel.from_xml_string(
    '<mujoco><worldbody><body><joint type="hinge"/><geom type="sphere" size="0.01"/></body></worldbody></mujoco>'
)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
print(
    f"planning: arch={platform.machine()}, numpy={numpy.__version__}, "
    f"mujoco={mujoco.__version__}, pinocchio={pinocchio.__version__}, "
    "headless_model_check=PASS"
)
PY
"$repo_dir/.vision-venv/bin/python" - <<'PY'
import cv2, ftfy, numpy, regex, torch, torchvision, ultralytics
print(f"vision: numpy={numpy.__version__}, cv2={cv2.__version__}")
print(f"torch={torch.__version__}, torchvision={torchvision.__version__}, ultralytics={ultralytics.__version__}")
if not torch.cuda.is_available():
    raise SystemExit("Bundled PyTorch cannot access the Jetson CUDA device")
print(f"gpu={torch.cuda.get_device_name(0)}")
PY

touch "$repo_dir/.offline-install.complete"
echo "Offline GraspV2 installation complete"
echo "AimDK overlay: $aimdk_setup"
echo "Run: ./offline_run.sh --target-class cup --plan-only"
