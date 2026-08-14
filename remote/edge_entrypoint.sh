#!/usr/bin/env bash
set -euo pipefail

edge_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: remote/edge_entrypoint.sh capture [capture options]
       remote/edge_entrypoint.sh preflight --bundle DIR
       remote/edge_entrypoint.sh execute --bundle DIR --confirm-control-authority
       remote/edge_entrypoint.sh diagnose

This entry point runs only on the X2 Orin. It discovers and sources the
firmware-matched AimDK installation before touching camera or control APIs.
EOF
}

if (($# == 0)); then
  usage >&2
  exit 2
fi

operation="$1"
shift
case "$operation" in
  capture|preflight|execute|diagnose) ;;
  -h|--help) usage; exit 0 ;;
  *) echo "Unknown edge operation: $operation" >&2; usage >&2; exit 2 ;;
esac

if [[ "$(uname -m)" != "aarch64" && "${GRASPV2_ALLOW_NON_ARM_EDGE:-0}" != "1" ]]; then
  echo "The robot edge runtime requires aarch64; found $(uname -m)" >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

aimdk_setup="$($edge_root/tools/find_aimdk_setup.sh)"
restore_nounset=false
if [[ "$-" == *u* ]]; then
  restore_nounset=true
  set +u
fi
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "$aimdk_setup"
if [[ "$restore_nounset" == true ]]; then
  set -u
fi

export PYTHONNOUSERSITE=1
export PYTHONPATH="$edge_root${PYTHONPATH:+:$PYTHONPATH}"
export GRASPV2_X2_ENV_READY=1
export GRASPV2_RUNTIME_PROFILE=competition
export GRASPV2_AIMDK_SETUP="$aimdk_setup"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0

if [[ "$operation" == "diagnose" ]]; then
  echo "edge_root=$edge_root"
  echo "architecture=$(uname -m)"
  echo "aimdk_setup=$aimdk_setup"
  /usr/bin/python3 - <<'PY'
import cv2
import rclpy
import aimdk_msgs
import sensor_msgs
print("robot_edge_imports=PASS")
print(f"opencv={cv2.__version__}")
PY
  exit 0
fi

case "$operation" in
  capture)
    exec /usr/bin/python3 "$edge_root/vision/ros_rgbd_capture.py" "$@"
    ;;
  preflight)
    exec /usr/bin/python3 "$edge_root/remote/execute_bundle.py" \
      --preflight "$@"
    ;;
  execute)
    exec /usr/bin/python3 "$edge_root/remote/execute_bundle.py" \
      --execute "$@"
    ;;
esac
