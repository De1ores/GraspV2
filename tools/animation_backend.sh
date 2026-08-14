#!/usr/bin/env bash
# Internal live-robot backend. Users enter through ./run.sh only.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
animation=""
robot_user="${GRASPV2_ROBOT_USER:-agi}"
robot_source_address="${GRASPV2_ROBOT_SOURCE_ADDRESS:-}"
robot_password="${GRASPV2_ROBOT_PASSWORD:-1}"
primary_robot_address="${GRASPV2_ROBOT_ADDRESS:-10.0.200.40}"
fallback_robot_address="${GRASPV2_ROBOT_FALLBACK_ADDRESS:-10.0.1.40}"
robot_animation="/tmp/graspv2_mc_animation.csv"
runtime_profile="$($repo_dir/tools/detect_runtime_profile.sh)"
assume_yes=false
initial_gripper_position=""
initial_gripper_duration=""
gripper_events=()

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
    --initial-gripper-position)
      [[ $# -ge 2 ]] || { echo "--initial-gripper-position requires a value" >&2; exit 2; }
      initial_gripper_position="$2"
      shift 2
      ;;
    --initial-gripper-duration)
      [[ $# -ge 2 ]] || { echo "--initial-gripper-duration requires a value" >&2; exit 2; }
      initial_gripper_duration="$2"
      shift 2
      ;;
    --gripper-event)
      [[ $# -ge 2 ]] || { echo "--gripper-event requires TIME:POSITION:LABEL" >&2; exit 2; }
      gripper_events+=("$2")
      shift 2
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

robot_ssh() {
  local address="$1"
  shift
  local source_address="$robot_source_address"
  local bind_args=()
  if [[ -z "$source_address" ]]; then
    source_address="$(ip -4 route get "$address" 2>/dev/null | sed -n 's/.* src \([^ ]*\).*/\1/p' | head -n 1)"
  fi
  if [[ -n "$source_address" ]]; then
    bind_args=(-b "$source_address")
  fi
  setsid -w ssh \
    "${bind_args[@]}" \
    -o BatchMode=no \
    -o ConnectTimeout=5 \
    -o ConnectionAttempts=1 \
    -o NumberOfPasswordPrompts=1 \
    -o StrictHostKeyChecking=accept-new \
    "$robot_user@$address" \
    "$@"
}

robot_address=""
if [[ "$runtime_profile" == "test" ]]; then
  askpass_helper="$repo_dir/tools/ssh_askpass.sh"
  [[ -x "$askpass_helper" ]] || {
    echo "SSH askpass helper is missing or not executable: $askpass_helper" >&2
    exit 1
  }
  export GRASPV2_ROBOT_PASSWORD="$robot_password"
  export SSH_ASKPASS="$askpass_helper"
  export SSH_ASKPASS_REQUIRE=force
  export DISPLAY="${DISPLAY:-graspv2:0}"

  robot_addresses=("$primary_robot_address")
  if [[ -n "$fallback_robot_address" && "$fallback_robot_address" != "$primary_robot_address" ]]; then
    robot_addresses+=("$fallback_robot_address")
  fi
  for candidate_address in "${robot_addresses[@]}"; do
    echo "Checking test robot SSH: $robot_user@$candidate_address"
    if robot_ssh "$candidate_address" true </dev/null; then
      robot_address="$candidate_address"
      break
    fi
    echo "Test robot SSH unavailable at $candidate_address; trying fallback." >&2
  done
  if [[ -z "$robot_address" ]]; then
    echo "Test robot SSH unavailable at all configured addresses: ${robot_addresses[*]}" >&2
    exit 1
  fi
  echo "Test robot SSH selected: $robot_user@$robot_address"
else
  [[ "$(uname -m)" == "aarch64" && "$(id -un)" == "agi" ]] || {
    echo "Competition profile requires local agi/aarch64 execution" >&2
    exit 1
  }
  echo "Competition robot selected: local agi/aarch64 MC"
fi

gripper_args=(--omnipicker-sdk "$repo_dir/omnipicker_hand_student.py")
if [[ "$runtime_profile" == "competition" ]]; then
  gripper_args+=(--require-gripper-sdk)
fi
if [[ -n "$initial_gripper_position" ]]; then
  gripper_args+=(--initial-gripper-position "$initial_gripper_position")
fi
if [[ -n "$initial_gripper_duration" ]]; then
  gripper_args+=(--initial-gripper-duration "$initial_gripper_duration")
fi
for event in "${gripper_events[@]}"; do
  gripper_args+=(--gripper-event "$event")
done

installed_node="$repo_dir/install/graspv2/lib/graspv2/x2_mc_custom_grasp"
if [[ ! -x "$installed_node" ]] ||
   [[ "$repo_dir/graspv2/mc_custom_grasp.py" -nt "$installed_node" ]] ||
   [[ "$repo_dir/setup.py" -nt "$installed_node" ]]; then
  echo "Building the compatible AimDK animation interface..."
  "$repo_dir/tools/build_graspv2_with_installed_aimdk.sh"
fi

# shellcheck source=/dev/null
source "$repo_dir/tools/setup_x2_mc_env.sh"

echo "Live step 1/3: read-only MC and arm preflight"
ros2 run graspv2 x2_mc_custom_grasp \
  --animation "$animation" \
  --robot-animation-path "$robot_animation" \
  "${gripper_args[@]}" \
  --preflight

local_checksum="$(sha256sum -- "$animation")"
local_checksum="${local_checksum%% *}"

echo
echo "Ready to play through MC animation_player:"
echo "  local CSV:  $animation"
echo "  SHA-256:    $local_checksum"
if [[ "$runtime_profile" == "competition" ]]; then
  echo "  environment: competition robot (local)"
  echo "  robot file: local:$robot_animation"
else
  echo "  environment: test robot (SSH fallback path)"
  echo "  robot file: $robot_user@$robot_address:$robot_animation"
fi
echo "  profile:    ultra (14 physical arm joints)"
echo "  gripper:    repository omnipicker_hand_student.py"
echo "  MC remains in control of balance; the CSV contains its return path."
if [[ "$assume_yes" != true ]]; then
  read -r -p "Type RUN (case-insensitive) to stage the CSV and start motion: " confirmation
  if [[ "${confirmation^^}" != "RUN" ]]; then
    echo "Cancelled; no robot file or motion was changed."
    exit 0
  fi
fi

if [[ "$runtime_profile" == "competition" ]]; then
  echo "Live step 2/3: install local volatile CSV and verify checksum"
  temporary_animation="${robot_animation}.tmp.$$"
  trap 'rm -f -- "$temporary_animation"' EXIT
  install -m 0644 "$animation" "$temporary_animation"
  mv -f -- "$temporary_animation" "$robot_animation"
  trap - EXIT
  remote_checksum_line="$(
    sha256sum "$robot_animation"
    stat -c %a "$robot_animation"
  )"
else
  echo "Live step 2/3: upload test-robot volatile CSV and verify checksum"
  remote_checksum_line="$(
    robot_ssh \
      "$robot_address" \
      "umask 022; dd of=$robot_animation status=none; chmod 0644 $robot_animation; sha256sum $robot_animation; stat -c %a $robot_animation" \
      < "$animation"
  )"
fi
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
  "${gripper_args[@]}" \
  --execute
