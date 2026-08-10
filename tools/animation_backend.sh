#!/usr/bin/env bash
# Internal live-robot backend. Users enter through ./run.sh only.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
animation=""
robot_address="${GRASPV2_ROBOT_ADDRESS:-10.0.200.40}"
robot_user="${GRASPV2_ROBOT_USER:-agi}"
robot_source_address="${GRASPV2_ROBOT_SOURCE_ADDRESS:-}"
robot_animation="/tmp/graspv2_mc_animation.csv"
assume_yes=false

while (($#)); do
  case "$1" in
    --animation)
      [[ $# -ge 2 ]] || { echo "--animation requires a path" >&2; exit 2; }
      animation="$2"
      shift 2
      ;;
    --yes)
      assume_yes=true
      shift
      ;;
    *)
      echo "Unknown internal animation option: $1" >&2
      exit 2
      ;;
  esac
done

[[ -n "$animation" && -f "$animation" ]] || {
  echo "MC animation CSV not found: $animation" >&2
  exit 1
}
animation="$(realpath -- "$animation")"

installed_node="$repo_dir/install/graspv2/lib/graspv2/x2_mc_custom_grasp"
if [[ ! -x "$installed_node" ]] ||
   [[ "$repo_dir/graspv2/mc_custom_grasp.py" -nt "$installed_node" ]] ||
   [[ "$repo_dir/setup.py" -nt "$installed_node" ]]; then
  echo "Building the verified v0.9 AimDK animation interface..."
  "$repo_dir/tools/build_graspv2_with_installed_aimdk.sh"
fi

# shellcheck source=/dev/null
source "$repo_dir/tools/setup_x2_mc_env.sh"

echo "Live step 1/3: read-only MC and arm preflight"
ros2 run graspv2 x2_mc_custom_grasp \
  --animation "$animation" \
  --robot-animation-path "$robot_animation" \
  --preflight

local_checksum="$(sha256sum -- "$animation")"
local_checksum="${local_checksum%% *}"
if [[ -z "$robot_source_address" ]]; then
  robot_source_address="$(ip -4 route get "$robot_address" 2>/dev/null | sed -n 's/.* src \([^ ]*\).*/\1/p' | head -n 1)"
fi
ssh_bind_args=()
if [[ -n "$robot_source_address" ]]; then
  ssh_bind_args=(-b "$robot_source_address")
fi

echo
echo "Ready to play through MC animation_player:"
echo "  local CSV:  $animation"
echo "  SHA-256:    $local_checksum"
echo "  robot file: $robot_user@$robot_address:$robot_animation"
echo "  profile:    ultra (14 physical arm joints)"
echo "  MC remains in control of balance; the CSV contains its return path."
if [[ "$assume_yes" != true ]]; then
  read -r -p "Type RUN (case-insensitive) to upload and start motion: " confirmation
  if [[ "${confirmation^^}" != "RUN" ]]; then
    echo "Cancelled; no robot file or motion was changed."
    exit 0
  fi
fi

echo "Live step 2/3: upload volatile CSV and verify checksum"
remote_checksum_line="$(
  ssh \
    "${ssh_bind_args[@]}" \
    -o ConnectTimeout=5 \
    -o StrictHostKeyChecking=accept-new \
    "$robot_user@$robot_address" \
    "umask 022; dd of=$robot_animation status=none; chmod 0644 $robot_animation; sha256sum $robot_animation; stat -c %a $robot_animation" \
    < "$animation"
)"
remote_checksum="${remote_checksum_line%% *}"
remote_mode="${remote_checksum_line##*$'\n'}"
[[ "$remote_checksum" == "$local_checksum" ]] || {
  echo "CSV checksum mismatch after upload" >&2
  exit 1
}
[[ "$remote_mode" == "644" ]] || {
  echo "CSV mode mismatch: expected 644, got $remote_mode" >&2
  exit 1
}

echo "Live step 3/3: request one non-interrupting ani_path playback"
ros2 run graspv2 x2_mc_custom_grasp \
  --animation "$animation" \
  --robot-animation-path "$robot_animation" \
  --execute
