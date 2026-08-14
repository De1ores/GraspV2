#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=robot_ssh_transport.sh
source "$repo_dir/tools/robot_ssh_transport.sh"

robot_host="${GRASPV2_ROBOT_HOST:-${GRASPV2_ROBOT_ADDRESS:-}}"
remote_root="${GRASPV2_REMOTE_ROOT:-/home/agi/graspv2-edge}"
remote_output="/tmp/graspv2-edge-capture-$(date +%s)-$$"
output_dir="$repo_dir/output"
capture_args=()

usage() {
  cat <<'EOF'
Usage: tools/remote_capture.sh --robot-host IP [capture options]

Run the official X2 RGB-D topic subscriber on agi@IP and copy color.png,
depth.png and camera.json back to the local output directory. Supported
capture options: --output-dir, --timeout, --warmup-frames, --max-skew-ms,
--depth-scale-m, --image-rotation-deg and the four topic overrides.
EOF
}

while (($#)); do
  case "$1" in
    --robot-host)
      (($# >= 2)) || { echo "$1 requires a value" >&2; exit 2; }
      robot_host="$2"
      shift 2
      ;;
    --output-dir)
      (($# >= 2)) || { echo "$1 requires a value" >&2; exit 2; }
      output_dir="$2"
      shift 2
      ;;
    --timeout|--warmup-frames|--max-skew-ms|--depth-scale-m|--image-rotation-deg|--color-topic|--depth-topic|--camera-info-topic|--depth-camera-info-topic|--hardware-config)
      (($# >= 2)) || { echo "$1 requires a value" >&2; exit 2; }
      if [[ "$1" != "--hardware-config" ]]; then
        capture_args+=("$1" "$2")
      fi
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown remote capture option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$robot_host" ]] || {
  echo "--robot-host is required (or set GRASPV2_ROBOT_HOST)" >&2
  exit 2
}
if [[ "$remote_root" == "/home/agi/" || \
      "$remote_root" != /home/agi/* || \
      "$remote_root" == *[!A-Za-z0-9_./-]* || \
      "$remote_root" == *"//"* || \
      "$remote_root" == *"/../"* || "$remote_root" == *"/.." || \
      "$remote_root" == *"/./"* || "$remote_root" == *"/." ]]; then
  echo "GRASPV2_REMOTE_ROOT must be a simple absolute path below /home/agi" >&2
  exit 2
fi
mkdir -p "$output_dir"

echo "Capturing X2 RGB-D on $(graspv2_robot_target "$robot_host")..."
graspv2_robot_ssh "$robot_host" \
  "$remote_root/remote/edge_entrypoint.sh" capture \
  --output-dir "$remote_output" \
  --hardware-config "$remote_root/config/x2_aimdk_hardware.json" \
  "${capture_args[@]}"

temporary_dir="$(mktemp -d "$output_dir/.remote-capture.XXXXXX")"
cleanup() {
  if [[ -d "$temporary_dir" ]]; then
    rm -f -- \
      "$temporary_dir/color.png" \
      "$temporary_dir/depth.png" \
      "$temporary_dir/camera.json"
    rmdir -- "$temporary_dir" 2>/dev/null || true
  fi
}
trap cleanup EXIT
for filename in color.png depth.png camera.json; do
  graspv2_robot_scp_from \
    "$robot_host" "$remote_output/$filename" "$temporary_dir/$filename"
done

/usr/bin/python3 - "$temporary_dir/camera.json" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
document = json.loads(path.read_text(encoding="utf-8"))
connection = document.get("device", {}).get("connection_type")
if connection != "ros2_aimdk":
    raise SystemExit(f"unexpected remote RGB-D source: {connection!r}")
for name in ("color", "depth"):
    topic = document.get("ros_topics", {}).get(name)
    if not isinstance(topic, str) or not topic.startswith("/aima/hal/sensor/"):
        raise SystemExit(f"invalid AimDK {name} topic in camera.json: {topic!r}")
print("Remote RGB-D metadata gate: PASS")
PY

for filename in color.png depth.png camera.json; do
  mv -f -- "$temporary_dir/$filename" "$output_dir/$filename"
done
trap - EXIT
rmdir -- "$temporary_dir"
echo "Remote RGB-D copied to: $(realpath -- "$output_dir")"
