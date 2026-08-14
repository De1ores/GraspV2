#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
robot_host="${GRASPV2_ROBOT_HOST:-${GRASPV2_ROBOT_ADDRESS:-}}"
target_class=""
execute=false
confirm_calibrated=false
assume_yes=false
pipeline_args=()

usage() {
  cat <<'EOF'
Usage: ./run_split_grasp_pipeline.sh --robot-host IP --target-class NAME [options]

Distributed X2 workflow:
  1. The Orin subscribes to the official AimDK RGB-D topics.
  2. RGB-D inference, depth geometry, official IK and MuJoCo run locally.
  3. A checksum-bound MC animation is uploaded and executed on the Orin.

Split options:
  --robot-host IP          Robot SSH/AimDK host; SSH user defaults to agi.
  --execute                Run robot preflight and final animation.
  --confirm-calibrated     Required for --execute.
  --yes                    Skip the final RUN prompt.

All normal recognition/planning options from run_full_grasp_pipeline.sh are
accepted, except capture/execution backend overrides. Without --execute this
command stops after creating output/edge_execution/manifest.json.

Important: split execution uses one atomic MC animation with its return path.
It cannot pause for the original two mid-motion visual verification gates.
EOF
}

while (($#)); do
  case "$1" in
    --robot-host)
      (($# >= 2)) || { echo "$1 requires a value" >&2; exit 2; }
      robot_host="$2"
      shift 2
      ;;
    --target-class)
      (($# >= 2)) || { echo "$1 requires a value" >&2; exit 2; }
      target_class="$2"
      pipeline_args+=("$1" "$2")
      shift 2
      ;;
    --execute) execute=true; shift ;;
    --confirm-calibrated) confirm_calibrated=true; shift ;;
    --yes) assume_yes=true; shift ;;
    --capture-backend|--verification-capture-backend|--use-existing-vision|--plan-only)
      echo "$1 is managed by the split pipeline and must not be supplied" >&2
      exit 2
      ;;
    -h|--help) usage; exit 0 ;;
    *)
      pipeline_args+=("$1")
      shift
      ;;
  esac
done

[[ -n "$robot_host" ]] || { echo "--robot-host is required" >&2; exit 2; }
[[ -n "$target_class" ]] || { echo "--target-class is required" >&2; exit 2; }
if [[ "$execute" == true && "$confirm_calibrated" != true ]]; then
  echo "--execute requires --confirm-calibrated after checking the robot camera extrinsics" >&2
  exit 2
fi
if [[ "$assume_yes" == true && "$execute" != true ]]; then
  echo "--yes is only valid with --execute" >&2
  exit 2
fi

export GRASPV2_ROBOT_HOST="$robot_host"
"$repo_dir/run_full_grasp_pipeline.sh" \
  "${pipeline_args[@]}" \
  --capture-backend x2-remote \
  --plan-only

if [[ -n "${GRASPV2_PLANNING_PYTHON:-}" ]]; then
  planning_python="$GRASPV2_PLANNING_PYTHON"
elif [[ -x "$repo_dir/.planning-venv/bin/python" ]]; then
  planning_python="$repo_dir/.planning-venv/bin/python"
else
  planning_python="$repo_dir/.venv/bin/python"
fi
[[ -x "$planning_python" ]] || { echo "Local planning Python is missing: $planning_python" >&2; exit 1; }

bundle_dir="$repo_dir/output/edge_execution"
PYTHONPATH="$repo_dir${PYTHONPATH:+:$PYTHONPATH}" \
  "$planning_python" -m graspv2.remote_bundle \
    --approach-trajectory "$repo_dir/output/planned_trajectory.json" \
    --lift-trajectory "$repo_dir/output/planned_lift.json" \
    --return-trajectory "$repo_dir/output/planned_return.json" \
    --initial-vision "$repo_dir/output/result.json" \
    --target-class "$target_class" \
    --output-dir "$bundle_dir"

if [[ "$execute" != true ]]; then
  echo "Split plan-only complete; no robot control interface was contacted."
  exit 0
fi

echo
echo "Robot-side read-only preflight..."
"$repo_dir/tools/remote_execute.sh" \
  --robot-host "$robot_host" --bundle "$bundle_dir" --preflight

if [[ "$assume_yes" != true ]]; then
  echo
  echo "The final step moves the right arm and OmniPicker on agi@$robot_host."
  echo "Split mode has no mid-motion visual gates. Type RUN to continue:"
  read -r confirmation
  [[ "$confirmation" == "RUN" ]] || { echo "Execution cancelled."; exit 1; }
fi

"$repo_dir/tools/remote_execute.sh" \
  --robot-host "$robot_host" --bundle "$bundle_dir" \
  --execute --confirm-control-authority
