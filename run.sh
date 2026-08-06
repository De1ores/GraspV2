#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
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
  echo "Installing planning dependencies (first run only)"
  (cd "$repo_dir" && "$python_bin" -m pip install -r requirements.txt)
fi

export PYTHONPATH="$repo_dir/x2_ik_sdk/src:$repo_dir${PYTHONPATH:+:$PYTHONPATH}"
exec "$python_bin" -m graspv2.cli "$@"
