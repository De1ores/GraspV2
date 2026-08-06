#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
venv_dir="${GRASPV2_VENV:-$repo_dir/.venv}"
torch_index="${GRASPV2_TORCH_INDEX:-https://pypi.jetson-ai-lab.io/jp6/cu126}"
torch_wheel="${GRASPV2_TORCH_WHEEL:-}"
torchvision_wheel="${GRASPV2_TORCHVISION_WHEEL:-}"
cusparselt_dir="${GRASPV2_CUSPARSELT_ROOT:-$venv_dir/cusparselt}"
cusparselt_archive="${GRASPV2_CUSPARSELT_ARCHIVE:-}"
cusparselt_url="${GRASPV2_CUSPARSELT_URL:-https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-aarch64/libcusparse_lt-linux-aarch64-0.7.1.0-archive.tar.xz}"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This setup script is only for an aarch64 NVIDIA Jetson host" >&2
  exit 1
fi
if ! /usr/bin/python3 -m venv --help >/dev/null 2>&1; then
  echo "python3.10-venv is missing; install it with apt before continuing" >&2
  exit 1
fi

if [[ ! -x "$venv_dir/bin/python" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$venv_dir"
fi

if [[ ! -e "$cusparselt_dir/lib/libcusparseLt.so.0" ]]; then
  if [[ -z "$cusparselt_archive" ]]; then
    if ! command -v curl >/dev/null 2>&1; then
      echo "curl is required to install the project-local cuSPARSELt runtime" >&2
      exit 1
    fi
    cusparselt_archive="$(mktemp --suffix=.tar.xz)"
    trap 'rm -f -- "$cusparselt_archive"' EXIT
    curl --fail --location --show-error --output "$cusparselt_archive" "$cusparselt_url"
  elif [[ ! -f "$cusparselt_archive" ]]; then
    echo "GRASPV2_CUSPARSELT_ARCHIVE does not exist: $cusparselt_archive" >&2
    exit 1
  fi
  mkdir -p "$cusparselt_dir"
  tar --extract --xz --file "$cusparselt_archive" \
    --directory "$cusparselt_dir" --strip-components=1
fi

export LD_LIBRARY_PATH="$cusparselt_dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

if "$venv_dir/bin/python" - <<'PY' >/dev/null 2>&1
import torch
import torchvision

assert torch.__version__.split("+")[0] == "2.8.0"
assert torchvision.__version__.split("+")[0] == "0.23.0"
PY
then
  echo "Reusing torch 2.8.0 and torchvision 0.23.0 from $venv_dir"
else
  if [[ -n "$torch_wheel" || -n "$torchvision_wheel" ]]; then
    if [[ ! -f "$torch_wheel" || ! -f "$torchvision_wheel" ]]; then
      echo "Both GRASPV2_TORCH_WHEEL and GRASPV2_TORCHVISION_WHEEL must name existing files" >&2
      exit 1
    fi
    "$venv_dir/bin/python" -m pip install --no-deps "$torch_wheel" "$torchvision_wheel"
  else
    "$venv_dir/bin/python" -m pip install --no-deps \
      torch==2.8.0 torchvision==0.23.0 \
      --index-url "$torch_index"
  fi
fi

if "$venv_dir/bin/python" - <<'PY' >/dev/null 2>&1
import ftfy
import numpy
import regex
import tqdm
import ultralytics
import wcwidth

assert numpy.__version__ == "1.26.4"
assert ultralytics.__version__ == "8.4.90"
assert ftfy.__version__ == "6.3.1"
assert regex.__version__ == "2026.7.19"
assert tqdm.__version__ == "4.67.1"
PY
then
  echo "Reusing the validated YOLOE Python dependencies from $venv_dir"
else
  "$venv_dir/bin/python" -m pip install \
    numpy==1.26.4 \
    ultralytics==8.4.90 \
    ftfy==6.3.1 \
    regex==2026.7.19 \
    tqdm==4.67.1 \
    wcwidth
fi

export YOLO_AUTOINSTALL=false
"$venv_dir/bin/python" - <<'PY'
import torch
import torchvision
import ultralytics

print(f"torch={torch.__version__} cuda={torch.version.cuda}")
print(f"torchvision={torchvision.__version__} ultralytics={ultralytics.__version__}")
if not torch.cuda.is_available():
    raise SystemExit("PyTorch cannot access the Orin CUDA device")
print(f"gpu={torch.cuda.get_device_name(0)}")
PY
