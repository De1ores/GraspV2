#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
planning_venv="${GRASPV2_PLANNING_VENV:-$repo_dir/.planning-venv}"
vision_venv="${GRASPV2_VISION_VENV:-$repo_dir/.vision-venv}"
python_command="${GRASPV2_LOCAL_PYTHON:-python3}"
torch_index_url="${GRASPV2_TORCH_INDEX_URL:-}"

usage() {
  cat <<'EOF'
Usage: tools/install_local_split.sh

Install the heavy planning and vision environments on the local workstation.
Optional environment variables:
  GRASPV2_LOCAL_PYTHON       Python executable (default: python3)
  GRASPV2_TORCH_INDEX_URL    PyTorch wheel index matching the local CUDA stack
  GRASPV2_PLANNING_VENV      Planning environment path
  GRASPV2_VISION_VENV        Vision environment path
EOF
}

if (($#)); then
  case "$1" in -h|--help) usage; exit 0 ;; *) usage >&2; exit 2 ;; esac
fi
command -v "$python_command" >/dev/null || { echo "Python is missing: $python_command" >&2; exit 1; }
command -v ssh >/dev/null || { echo "OpenSSH client is required" >&2; exit 1; }
command -v scp >/dev/null || { echo "OpenSSH scp is required" >&2; exit 1; }

prepare_venv() {
  local path="$1"
  if [[ ! -x "$path/bin/python" ]]; then
    "$python_command" -m venv "$path"
  fi
  "$path/bin/python" -m pip install --upgrade pip setuptools wheel
}

prepare_venv "$planning_venv"
"$planning_venv/bin/python" -m pip install \
  "$repo_dir/x2_ik_sdk" \
  'mujoco>=3.3,<4' \
  'numpy>=1.24'

prepare_venv "$vision_venv"
mapfile -t vision_packages < <(
  grep -Ev '^(torch|torchvision)==|^[[:space:]]*(#|$)' \
    "$repo_dir/offline/requirements-vision.txt"
)
torch_args=(torch==2.8.0 torchvision==0.23.0)
if [[ -n "$torch_index_url" ]]; then
  torch_args+=(--index-url "$torch_index_url")
fi
"$vision_venv/bin/python" -m pip install "${torch_args[@]}"
"$vision_venv/bin/python" -m pip install "${vision_packages[@]}"

PYTHONPATH="$repo_dir/x2_ik_sdk/src:$repo_dir" \
  "$planning_venv/bin/python" - <<'PY'
import mujoco
import numpy
import pinocchio
import x2_ik_sdk
print(f"planning=PASS numpy={numpy.__version__} mujoco={mujoco.__version__}")
PY
"$vision_venv/bin/python" - <<'PY'
import cv2
import torch
import ultralytics
print(f"vision=PASS opencv={cv2.__version__} torch={torch.__version__}")
print(f"cuda_available={torch.cuda.is_available()}")
PY
echo "Local split-compute installation complete"
