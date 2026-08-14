#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "$(uname -m)" != "aarch64" && "${GRASPV2_ALLOW_NON_ARM_EDGE:-0}" != "1" ]]; then
  echo "Remote split runtime must be installed on the X2 Orin (aarch64)" >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is missing: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
for required in \
  "$repo_dir/remote/edge_entrypoint.sh" \
  "$repo_dir/remote/execute_bundle.py" \
  "$repo_dir/vision/ros_rgbd_capture.py" \
  "$repo_dir/omnipicker_hand_student.py" \
  "$repo_dir/config/x2_aimdk_hardware.json"; do
  [[ -f "$required" ]] || { echo "Incomplete robot edge bundle: $required" >&2; exit 1; }
done

chmod 0755 \
  "$repo_dir/remote/edge_entrypoint.sh" \
  "$repo_dir/remote/execute_bundle.py" \
  "$repo_dir/vision/ros_rgbd_capture.py" \
  "$repo_dir/omnipicker_hand_student.py" \
  "$repo_dir/tools/find_aimdk_setup.sh"

aimdk_setup="$($repo_dir/tools/find_aimdk_setup.sh)"
echo "Robot edge files: $repo_dir"
echo "AimDK overlay: $aimdk_setup"
echo "Checking ROS, AimDK, OpenCV and NumPy imports..."
GRASPV2_AIMDK_SETUP="$aimdk_setup" \
  "$repo_dir/remote/edge_entrypoint.sh" diagnose
echo "Remote GraspV2 edge installation complete"
echo "No Torch, YOLO, MuJoCo, Pinocchio or X2 IK packages were installed."
