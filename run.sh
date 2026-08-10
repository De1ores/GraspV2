#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# WSLg may expose a D3D12 device even when GPU access is blocked.  MuJoCo's
# GLFW viewer can then crash during native driver teardown.  The software
# renderer is slower but deterministic for this small planning scene.  Set
# GRASPV2_WSL_SOFTWARE_GL=0 after confirming accelerated WSLg OpenGL works.
if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null && \
   [[ "${GRASPV2_WSL_SOFTWARE_GL:-1}" != "0" ]]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-llvmpipe}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
  export MUJOCO_GL="${MUJOCO_GL:-glfw}"
fi

if [[ -x "$repo_dir/.planning-venv/bin/python" ]]; then
  default_python="$repo_dir/.planning-venv/bin/python"
else
  default_python="$repo_dir/.venv/bin/python"
fi
python_bin="${GRASPV2_PYTHON:-$default_python}"

if [[ ! -x "$python_bin" ]]; then
  if [[ -n "${GRASPV2_PYTHON:-}" ]]; then
    echo "Configured GRASPV2_PYTHON is not executable: $python_bin" >&2
    exit 1
  fi
  echo "Creating isolated runtime environment in $repo_dir/.venv"
  python3 -m venv "$repo_dir/.venv"
fi

if ! PYTHONPATH="$repo_dir/x2_ik_sdk/src:$repo_dir${PYTHONPATH:+:$PYTHONPATH}" \
  "$python_bin" -c 'import mujoco, numpy, x2_ik_sdk' >/dev/null 2>&1; then
  if [[ "${GRASPV2_OFFLINE:-0}" == "1" ]]; then
    echo "Offline planning environment is incomplete: $python_bin" >&2
    echo "Run ./install_offline.sh from the complete deployment bundle." >&2
    exit 1
  fi
  echo "Installing planning dependencies (first run only)"
  (cd "$repo_dir" && "$python_bin" -m pip install -r requirements.txt)
fi

export PYTHONPATH="$repo_dir/x2_ik_sdk/src:$repo_dir${PYTHONPATH:+:$PYTHONPATH}"
exec "$python_bin" -m graspv2.cli "$@"
