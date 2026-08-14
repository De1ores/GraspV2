#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=robot_ssh_transport.sh
source "$repo_dir/tools/robot_ssh_transport.sh"

robot_host="${GRASPV2_ROBOT_HOST:-${GRASPV2_ROBOT_ADDRESS:-}}"
remote_root="${GRASPV2_REMOTE_ROOT:-/home/agi/graspv2-edge}"
remote_bundle="/tmp/graspv2-edge-execution-$(date +%s)-$$"
bundle=""
mode=""
confirmed=false

usage() {
  cat <<'EOF'
Usage: tools/remote_execute.sh --robot-host IP --bundle DIR --preflight
       tools/remote_execute.sh --robot-host IP --bundle DIR --execute --confirm-control-authority

Upload only manifest.json and grasp_animation.csv, then run AimDK preflight or
the final MC-owned motion locally on the robot Orin.
EOF
}

while (($#)); do
  case "$1" in
    --robot-host|--bundle)
      (($# >= 2)) || { echo "$1 requires a value" >&2; exit 2; }
      case "$1" in
        --robot-host) robot_host="$2" ;;
        --bundle) bundle="$2" ;;
      esac
      shift 2
      ;;
    --preflight|--execute)
      [[ -z "$mode" ]] || { echo "Choose only one of --preflight/--execute" >&2; exit 2; }
      mode="$1"
      shift
      ;;
    --confirm-control-authority) confirmed=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown remote execution option: $1" >&2; usage >&2; exit 2 ;;
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
[[ -n "$bundle" && -d "$bundle" ]] || { echo "--bundle must be a directory" >&2; exit 2; }
[[ -n "$mode" ]] || { echo "--preflight or --execute is required" >&2; exit 2; }
if [[ "$mode" == "--execute" && "$confirmed" != true ]]; then
  echo "--execute requires --confirm-control-authority" >&2
  exit 2
fi
if [[ "$mode" == "--preflight" && "$confirmed" == true ]]; then
  echo "--confirm-control-authority is only valid with --execute" >&2
  exit 2
fi
for filename in manifest.json grasp_animation.csv; do
  [[ -f "$bundle/$filename" ]] || { echo "Bundle file is missing: $bundle/$filename" >&2; exit 1; }
done

graspv2_robot_ssh "$robot_host" mkdir -p "$remote_bundle"
for filename in manifest.json grasp_animation.csv; do
  graspv2_robot_scp_to \
    "$robot_host" "$bundle/$filename" "$remote_bundle/$filename"
done

remote_args=(--bundle "$remote_bundle")
if [[ "$confirmed" == true ]]; then
  remote_args+=(--confirm-control-authority)
fi
echo "Robot-side ${mode#--}: $(graspv2_robot_target "$robot_host")"
graspv2_robot_ssh "$robot_host" \
  "$remote_root/remote/edge_entrypoint.sh" "${mode#--}" "${remote_args[@]}"
