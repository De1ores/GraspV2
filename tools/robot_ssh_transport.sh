#!/usr/bin/env bash
# Shared OpenSSH transport for GraspV2 split deployment scripts.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This helper must be sourced by a GraspV2 transport script." >&2
  exit 1
fi

graspv2_transport_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
graspv2_robot_user="${GRASPV2_ROBOT_USER:-agi}"
graspv2_robot_password="${GRASPV2_ROBOT_PASSWORD:-1}"
graspv2_robot_source_address="${GRASPV2_ROBOT_SOURCE_ADDRESS:-}"

export GRASPV2_ROBOT_PASSWORD="$graspv2_robot_password"
export SSH_ASKPASS="$graspv2_transport_root/tools/ssh_askpass.sh"
export SSH_ASKPASS_REQUIRE=force
export DISPLAY="${DISPLAY:-graspv2:0}"

graspv2_robot_target() {
  local address="$1"
  if [[ -z "$address" || "$address" == *[!0-9A-Za-z:.%-]* ]]; then
    echo "Invalid robot host/address: $address" >&2
    return 2
  fi
  printf '%s@%s\n' "$graspv2_robot_user" "$address"
}

graspv2_ssh_options() {
  printf '%s\n' \
    -o BatchMode=no \
    -o ConnectTimeout=8 \
    -o ConnectionAttempts=1 \
    -o NumberOfPasswordPrompts=1 \
    -o StrictHostKeyChecking=accept-new
  if [[ -n "$graspv2_robot_source_address" ]]; then
    printf '%s\n' -b "$graspv2_robot_source_address"
  fi
}

graspv2_robot_ssh() {
  local address="$1"
  shift
  local options=()
  mapfile -t options < <(graspv2_ssh_options)
  setsid -w ssh "${options[@]}" "$(graspv2_robot_target "$address")" "$@" </dev/null
}

graspv2_robot_scp_to() {
  local address="$1"
  local source="$2"
  local destination="$3"
  local options=()
  mapfile -t options < <(graspv2_ssh_options)
  setsid -w scp "${options[@]}" -- "$source" \
    "$(graspv2_robot_target "$address"):$destination" </dev/null
}

graspv2_robot_scp_from() {
  local address="$1"
  local source="$2"
  local destination="$3"
  local options=()
  mapfile -t options < <(graspv2_ssh_options)
  setsid -w scp "${options[@]}" -- \
    "$(graspv2_robot_target "$address"):$source" "$destination" </dev/null
}
