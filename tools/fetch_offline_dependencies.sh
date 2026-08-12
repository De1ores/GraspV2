#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
planning_dir="$repo_dir/offline/wheelhouse/aarch64/planning"
vision_dir="$repo_dir/offline/wheelhouse/aarch64/vision"
runtime_dir="$repo_dir/offline/runtime"
pip_cache="$repo_dir/offline/pip-cache"
mkdir -p "$planning_dir" "$vision_dir" "$runtime_dir" "$pip_cache"

download_checked() {
  local url="$1"
  local target="$2"
  local expected="$3"
  local partial="${target}.part"
  if [[ -f "$target" ]] && printf '%s  %s\n' "$expected" "$target" | sha256sum --check --status; then
    echo "Reusing $(basename "$target")"
    return
  fi
  if [[ -f "$target" && ! -e "$partial" ]]; then
    mv -- "$target" "$partial"
  else
    rm -f -- "$target"
  fi
  if ! curl --fail --location --retry 20 --retry-all-errors \
    --continue-at - --output "$partial" "$url"; then
    if printf '%s  %s\n' "$expected" "$partial" | sha256sum --check --status; then
      mv -- "$partial" "$target"
      return
    fi
    echo "Download interrupted; partial file retained for the next run: $partial" >&2
    exit 1
  fi
  if ! printf '%s  %s\n' "$expected" "$partial" | sha256sum --check --status; then
    rm -f -- "$partial"
    echo "Checksum mismatch: $target; discarded the invalid partial file" >&2
    exit 1
  fi
  mv -- "$partial" "$target"
}

platform_args=(
  --platform manylinux_2_28_aarch64
  --platform manylinux_2_17_aarch64
  --platform manylinux2014_aarch64
  --platform linux_aarch64
  --python-version 310
  --implementation cp
  --abi cp310
  --only-binary=:all:
)
pip_download=(
  python3 -m pip --disable-pip-version-check
  --cache-dir "$pip_cache" download --timeout 120 --retries 20
)

"${pip_download[@]}" --dest "$planning_dir" "${platform_args[@]}" \
  numpy==2.2.6 mujoco==3.11.0 pin==4.1.0 tomli==2.2.1
x2_wheel_stage="$(mktemp -d)"
trap 'rm -rf -- "$x2_wheel_stage"' EXIT
python3 -m pip --disable-pip-version-check wheel --no-deps --no-build-isolation \
  --wheel-dir "$x2_wheel_stage" "$repo_dir/x2_ik_sdk"
x2_wheels=("$x2_wheel_stage"/x2_ik_sdk-*.whl)
if [[ ${#x2_wheels[@]} -ne 1 || ! -f "${x2_wheels[0]}" ]]; then
  echo "Expected exactly one freshly built x2_ik_sdk wheel" >&2
  exit 1
fi
mv -- "${x2_wheels[0]}" "$planning_dir/$(basename "${x2_wheels[0]}")"
rm -rf -- "$x2_wheel_stage"
trap - EXIT

download_checked \
  "https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/torch-2.8.0-cp310-cp310-linux_aarch64.whl" \
  "$vision_dir/torch-2.8.0-cp310-cp310-linux_aarch64.whl" \
  "62a1beee9f2f147076a974d2942c90060c12771c94740830327cae705b2595fc"
download_checked \
  "https://pypi.jetson-ai-lab.io/jp6/cu126/+f/907/c4c1933789645/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl" \
  "$vision_dir/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl" \
  "907c4c1933789645ebb20dd9181d40f8647978e6bd30086ae7b01febb937d2d1"
download_checked \
  "https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-aarch64/libcusparse_lt-linux-aarch64-0.7.1.0-archive.tar.xz" \
  "$runtime_dir/libcusparse_lt-linux-aarch64-0.7.1.0-archive.tar.xz" \
  "d3b0a660fd552e0bd9a4491b15299d968674833483d5f164cfea35e70646136c"
download_checked \
  "https://github.com/ultralytics/assets/releases/download/v8.4.0/mobileclip2_b.ts" \
  "$repo_dir/mobileclip2_b.ts" \
  "35d7f213e4d75f38514e4656ad3cb91158bd33e3805d8ac349f23b186f66982f"
download_checked \
  "https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26s-seg.pt" \
  "$repo_dir/yoloe-26s-seg.pt" \
  "48f24206bc8680d60cbbfa296b0140da849669b9515058b72f5a945142df0654"

"${pip_download[@]}" --dest "$vision_dir" --no-deps \
  ultralytics==8.4.90 ultralytics-thop==2.0.18
"${pip_download[@]}" --dest "$vision_dir" "${platform_args[@]}" \
  numpy==1.26.4 \
  opencv-python-headless==4.11.0.86 \
  ftfy==6.3.1 regex==2026.7.19 tqdm==4.67.1 wcwidth==0.2.13 \
  matplotlib==3.10.9 pillow==12.3.0 pyyaml==6.0.3 requests==2.34.2 \
  psutil==7.2.2 polars==1.43.2 nvidia-ml-py==13.610.43 \
  nvidia-cuda-cupti-cu12==12.6.80 \
  setuptools==80.9.0 \
  filelock==3.32.2 typing-extensions==4.16.0 sympy==1.14.0 \
  networkx==3.4.2 jinja2==3.1.6 fsspec==2026.7.0

echo "Offline dependencies are ready"
python3 "$repo_dir/tools/check_offline_assets.py" --root "$repo_dir"
du -sh "$planning_dir" "$vision_dir" "$runtime_dir"
