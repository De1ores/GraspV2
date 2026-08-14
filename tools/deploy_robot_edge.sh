#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=robot_ssh_transport.sh
source "$repo_dir/tools/robot_ssh_transport.sh"

robot_host="${GRASPV2_ROBOT_HOST:-${GRASPV2_ROBOT_ADDRESS:-}}"
remote_root="${GRASPV2_REMOTE_ROOT:-/home/agi/graspv2-edge}"

usage() {
  cat <<'EOF'
Usage: tools/deploy_robot_edge.sh --robot-host IP

Deploy the minimal capture/execution runtime to agi@IP. The default password
is read by OpenSSH askpass from GRASPV2_ROBOT_PASSWORD (default: 1).
EOF
}

while (($#)); do
  case "$1" in
    --robot-host)
      (($# >= 2)) || { echo "$1 requires a value" >&2; exit 2; }
      robot_host="$2"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown deploy option: $1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ -n "$robot_host" ]] || { echo "--robot-host is required" >&2; exit 2; }
if [[ "$remote_root" == "/home/agi/" || \
      "$remote_root" != /home/agi/* || \
      "$remote_root" == *[!A-Za-z0-9_./-]* || \
      "$remote_root" == *"//"* || \
      "$remote_root" == *"/../"* || "$remote_root" == *"/.." || \
      "$remote_root" == *"/./"* || "$remote_root" == *"/." ]]; then
  echo "GRASPV2_REMOTE_ROOT must be a simple absolute path below /home/agi" >&2
  exit 2
fi

archive="$(mktemp --suffix=.graspv2-edge.tar.gz)"
trap 'rm -f -- "$archive"' EXIT
files=(
  config/x2_aimdk_hardware.json
  graspv2/__init__.py
  graspv2/edge_manifest.py
  graspv2/hardware_contract.py
  graspv2/mc_animation.py
  graspv2/mc_custom_grasp.py
  graspv2/robot_profiles.py
  graspv2/ros_logging.py
  graspv2/trajectory.py
  omnipicker_hand_student.py
  remote/edge_entrypoint.sh
  remote/execute_bundle.py
  tools/find_aimdk_setup.sh
  tools/install_remote_split.sh
  vision/ros_rgbd_capture.py
)
tar -C "$repo_dir" -czf "$archive" "${files[@]}"
local_checksum="$(sha256sum "$archive")"
local_checksum="${local_checksum%% *}"

echo "Deploying minimal edge runtime to $(graspv2_robot_target "$robot_host"):$remote_root"
echo "Archive SHA-256: $local_checksum"
remote_archive="/tmp/graspv2-edge-$local_checksum.tar.gz"
graspv2_robot_ssh "$robot_host" mkdir -p "$remote_root"
graspv2_robot_scp_to "$robot_host" "$archive" "$remote_archive"
remote_checksum_line="$(
  graspv2_robot_ssh "$robot_host" sha256sum "$remote_archive"
)"
remote_checksum="${remote_checksum_line%% *}"
[[ "$remote_checksum" == "$local_checksum" ]] || {
  echo "Robot edge archive checksum mismatch after upload" >&2
  exit 1
}
graspv2_robot_ssh "$robot_host" \
  tar -xzf "$remote_archive" -C "$remote_root"
graspv2_robot_ssh "$robot_host" \
  "$remote_root/tools/install_remote_split.sh"
echo "Robot edge deployment complete: $remote_root"
