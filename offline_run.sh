#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f "$repo_dir/.offline-install.complete" ]]; then
  "$repo_dir/install_offline.sh"
fi

[[ -x "$repo_dir/.planning-venv/bin/python" ]] || {
  echo "Planning environment is missing; run ./install_offline.sh --force" >&2
  exit 1
}
[[ -x "$repo_dir/.vision-venv/bin/python" ]] || {
  echo "Vision environment is missing; run ./install_offline.sh --force" >&2
  exit 1
}

export GRASPV2_OFFLINE=1
export GRASPV2_PLANNING_PYTHON="$repo_dir/.planning-venv/bin/python"
export GRASPV2_VISION_PYTHON="$repo_dir/.vision-venv/bin/python"
export GRASPV2_CUSPARSELT_ROOT="$repo_dir/.runtime/cusparselt"
cupti_lib="$repo_dir/.vision-venv/lib/python3.10/site-packages/nvidia/cuda_cupti/lib"
export LD_LIBRARY_PATH="$GRASPV2_CUSPARSELT_ROOT/lib:$cupti_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PIP_NO_INDEX=1
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PYTHONNOUSERSITE=1
export YOLO_AUTOINSTALL=false
export YOLO_OFFLINE=true
export YOLO_CONFIG_DIR="$repo_dir/.runtime/ultralytics"
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export NO_ALBUMENTATIONS_UPDATE=1

# Load ROS, AimDK and the project overlay before RGB-D capture, planning or
# hardware preflight.  Child scripts inherit this exact offline environment.
# shellcheck source=/dev/null
source "$repo_dir/tools/setup_x2_mc_env.sh"

exec "$repo_dir/run_full_grasp_pipeline.sh" "$@"
