#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if_available=false

usage() {
  cat <<'EOF'
Usage: ./tools/build_orbbec_capture.sh [--if-available]

Build the direct Orbbec SDK RGB-D capture helper. The SDK is discovered from
GRASPV2_ORBBEC_SDK_ROOT, standard system prefixes, or a ROS 2 orbbec_camera
workspace. --if-available exits successfully when no SDK is installed.
EOF
}

while (($#)); do
  case "$1" in
    --if-available) if_available=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

prefixes=()
if [[ -n "${GRASPV2_ORBBEC_SDK_ROOT:-}" ]]; then
  prefixes+=("$GRASPV2_ORBBEC_SDK_ROOT")
fi
if [[ -n "${ORBBEC_SDK_ROOT:-}" ]]; then
  prefixes+=("$ORBBEC_SDK_ROOT")
fi
prefixes+=(
  /usr/local
  /opt/OrbbecSDK
  /opt/OrbbecSDK_v2
)
for ros_sdk in /home/*/ros2_ws/install/orbbec_camera; do
  [[ -d "$ros_sdk" ]] && prefixes+=("$ros_sdk")
done

include_dir="${GRASPV2_ORBBEC_INCLUDE_DIR:-}"
library_dir="${GRASPV2_ORBBEC_LIBRARY_DIR:-}"
for prefix in "${prefixes[@]}"; do
  if [[ -z "$include_dir" && -f "$prefix/include/libobsensor/ObSensor.hpp" ]]; then
    include_dir="$prefix/include"
  fi
  if [[ -z "$library_dir" && -e "$prefix/lib/libOrbbecSDK.so" ]]; then
    library_dir="$prefix/lib"
  fi
done

if [[ ! -f "$include_dir/libobsensor/ObSensor.hpp" || ! -e "$library_dir/libOrbbecSDK.so" ]]; then
  message="Orbbec SDK headers/library were not found; set GRASPV2_ORBBEC_SDK_ROOT"
  if [[ "$if_available" == true ]]; then
    echo "$message; SDK fallback will remain unavailable"
    exit 0
  fi
  echo "$message" >&2
  exit 1
fi

command -v g++ >/dev/null 2>&1 || {
  echo "g++ is required to build the Orbbec SDK capture helper" >&2
  exit 1
}
command -v pkg-config >/dev/null 2>&1 || {
  echo "pkg-config is required to locate OpenCV" >&2
  exit 1
}
pkg-config --exists opencv4 || {
  echo "OpenCV 4 development files are missing (pkg-config opencv4)" >&2
  exit 1
}

mkdir -p "$repo_dir/build"
read -r -a opencv_flags <<< "$(pkg-config --cflags --libs opencv4)"
opencv_system_include=()
# Some Jetson images retain an opencv4.pc that points at /usr/local after the
# headers moved back to the Ubuntu multiarch package under /usr/include.
if [[ -f /usr/include/opencv4/opencv2/imgcodecs.hpp ]]; then
  opencv_system_include=(-I/usr/include/opencv4)
fi

g++ -std=c++17 -O2 -Wall -Wextra \
  "$repo_dir/tools/orbbec_capture.cpp" \
  -o "$repo_dir/build/orbbec_capture" \
  -I"$include_dir" \
  "${opencv_system_include[@]}" \
  -L"$library_dir" -Wl,-rpath,"$library_dir" -lOrbbecSDK \
  "${opencv_flags[@]}"

echo "Built $repo_dir/build/orbbec_capture"
echo "Orbbec SDK: include=$include_dir library=$library_dir"
