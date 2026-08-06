#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
vision_result="$repo_dir/output/result.json"
if [[ -n "${GRASPV2_PLANNING_PYTHON:-}" ]]; then
  planning_python="$GRASPV2_PLANNING_PYTHON"
elif [[ -x "$repo_dir/.planning-venv/bin/python" ]]; then
  planning_python="$repo_dir/.planning-venv/bin/python"
else
  planning_python="$repo_dir/.venv/bin/python"
fi
target_class=""
robot=""
side="auto"
vision_conf="0.20"
table_clearance="0.025"
lift_height="0.10"
lift_duration="2.0"
capture_backend="x2-aimdk"
verification_capture_backend=""
camera_calibration=""
verification_timeout="45"
close_target_tolerance="0.08"
lifted_target_tolerance="0.10"
minimum_lift_ratio="0.60"
maximum_lateral_drift="0.08"
capture=true
plan_only=false
execute=false
confirm_calibrated=false
assume_yes=false

usage() {
  cat <<'EOF'
Usage: ./run_full_grasp_pipeline.sh --target-class NAME --robot youth|ultra [options]

Capture RGB-D, recognize exactly one requested object class, plan collision-
checked approach/lift paths, and prepare a visually verified grasp sequence.

Required:
  --target-class NAME       Object class/prompt, for example bottle, cup, or "game controller".
  --robot youth|ultra       Robot joint profile; youth=5 DoF/arm, ultra=7 DoF/arm.

Options:
  --side auto|left|right    Planning arm (default: auto; target side first, then fallback).
  --vision-conf VALUE       Detection threshold (default: 0.20).
  --table-clearance METERS  Required table clearance (default: 0.025).
  --lift-height METERS      Lift distance along the table normal (default: 0.10).
  --lift-duration SEC       Lift motion duration (default: 2.0).
  --capture-backend MODE    x2-aimdk (default), orbbec-sdk, or existing.
                            existing consumes a manually captured RGB-D frame.
  --verification-capture-backend MODE
                            Fresh checkpoint capture: x2-aimdk or orbbec-sdk.
                            Defaults to the capture backend, or x2-aimdk when
                            the initial input uses existing files.
  --camera-calibration PATH Camera-to-MuJoCo calibration for the selected sensor.
  --verification-timeout SEC
                            Maximum time for each live visual gate (default: 45).
  --close-target-tolerance METERS
                            Closed object-to-grasp-region limit (default: 0.08).
  --lifted-target-tolerance METERS
                            Lifted object-to-expected-target limit (default: 0.10).
  --minimum-lift-ratio VALUE
                            Required observed fraction of lift (default: 0.60).
  --maximum-lateral-drift METERS
                            Allowed object drift during lift (default: 0.08).
  --use-existing-vision     Reuse output/result.json after verifying its class.
  --plan-only               Stop after recognition and collision-checked planning.
  --execute                 Execute only after calibration confirmation and typing RUN.
  --confirm-calibrated      Required with --execute; confirms real extrinsics were checked.
  --yes                     Skip the final RUN prompt; only valid with --execute.
  -h, --help                Show this help.

Without --execute, no robot control interface is contacted.
EOF
}

while (($#)); do
  case "$1" in
    --target-class|--robot|--side|--vision-conf|--table-clearance|--lift-height|--lift-duration|--capture-backend|--verification-capture-backend|--camera-calibration|--verification-timeout|--close-target-tolerance|--lifted-target-tolerance|--minimum-lift-ratio|--maximum-lateral-drift)
      (($# >= 2)) || { echo "$1 requires a value" >&2; exit 2; }
      case "$1" in
        --target-class) target_class="$2" ;;
        --robot) robot="$2" ;;
        --side) side="$2" ;;
        --vision-conf) vision_conf="$2" ;;
        --table-clearance) table_clearance="$2" ;;
        --lift-height) lift_height="$2" ;;
        --lift-duration) lift_duration="$2" ;;
        --capture-backend) capture_backend="$2" ;;
        --verification-capture-backend) verification_capture_backend="$2" ;;
        --camera-calibration) camera_calibration="$2" ;;
        --verification-timeout) verification_timeout="$2" ;;
        --close-target-tolerance) close_target_tolerance="$2" ;;
        --lifted-target-tolerance) lifted_target_tolerance="$2" ;;
        --minimum-lift-ratio) minimum_lift_ratio="$2" ;;
        --maximum-lateral-drift) maximum_lateral_drift="$2" ;;
      esac
      shift 2
      ;;
    --use-existing-vision)
      capture=false
      shift
      ;;
    --plan-only)
      plan_only=true
      shift
      ;;
    --execute)
      execute=true
      shift
      ;;
    --confirm-calibrated)
      confirm_calibrated=true
      shift
      ;;
    --yes)
      assume_yes=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "$target_class" ]] || { echo "--target-class is required" >&2; exit 2; }
case "$robot" in
  youth|ultra) ;;
  *) echo "--robot must be youth or ultra" >&2; exit 2 ;;
esac
case "$side" in
  auto|left|right) ;;
  *) echo "--side must be auto, left or right" >&2; exit 2 ;;
esac
case "$capture_backend" in
  x2-aimdk|orbbec-sdk|existing) ;;
  *) echo "--capture-backend must be x2-aimdk, orbbec-sdk, or existing" >&2; exit 2 ;;
esac
if [[ -z "$verification_capture_backend" ]]; then
  if [[ "$capture_backend" == "existing" ]]; then
    verification_capture_backend="x2-aimdk"
  else
    verification_capture_backend="$capture_backend"
  fi
fi
case "$verification_capture_backend" in
  x2-aimdk|orbbec-sdk) ;;
  *) echo "--verification-capture-backend must be x2-aimdk or orbbec-sdk" >&2; exit 2 ;;
esac
if [[ "$execute" == true && "$confirm_calibrated" != true ]]; then
  echo "Execution blocked: add --confirm-calibrated only after checking camera-to-robot calibration." >&2
  exit 2
fi
if [[ "$assume_yes" == true && "$execute" != true ]]; then
  echo "--yes is only valid with --execute" >&2
  exit 2
fi
if [[ ! -x "$planning_python" ]]; then
  echo "Planning environment is missing: $planning_python" >&2
  echo "Re-run the svt deployment setup before using the full pipeline." >&2
  exit 1
fi
export GRASPV2_PYTHON="$planning_python"

mkdir -p "$repo_dir/output"
trajectory="$repo_dir/output/planned_trajectory_${robot}.json"
planning_report="$repo_dir/output/planning_report_${robot}.json"
lift_trajectory="$repo_dir/output/planned_lift_${robot}.json"
lift_report="$repo_dir/output/planning_report_lift_${robot}.json"
grasp_status="$repo_dir/output/grasp_status.json"

echo "[1/3] RGB-D recognition: class=$target_class"
if [[ "$capture" == true ]]; then
  vision_args=(
    --capture-backend "$capture_backend"
    --classes "$target_class"
    --target-class "$target_class"
    --conf "$vision_conf"
    --device 0
  )
  if [[ -n "$camera_calibration" ]]; then
    vision_args+=(--calibration "$camera_calibration")
  fi
  "$repo_dir/run_vision.sh" "${vision_args[@]}"
elif [[ ! -f "$vision_result" ]]; then
  echo "Existing vision result is missing: $vision_result" >&2
  exit 1
fi

/usr/bin/python3 - "$vision_result" "$target_class" <<'PY'
import json
import sys

path, requested = sys.argv[1:]
with open(path, encoding="utf-8") as stream:
    document = json.load(stream)
selected = document.get("selected_detection")
actual = selected.get("class_name") if isinstance(selected, dict) else None
if not isinstance(actual, str) or actual.casefold() != requested.casefold():
    raise SystemExit(
        f"Vision result class mismatch: requested {requested!r}, result contains {actual!r}"
    )
print(f"Vision class gate passed: {actual}")
PY

echo
echo "[2/3] Official IK and MuJoCo gate: robot=$robot side=$side"
"$repo_dir/run.sh" \
  --robot "$robot" \
  --mode sim \
  --side "$side" \
  --headless \
  --vision-result "$vision_result" \
  --trajectory "$trajectory" \
  --report "$planning_report" \
  --table-clearance "$table_clearance" \
  --lift-trajectory "$lift_trajectory" \
  --lift-report "$lift_report" \
  --lift-height "$lift_height" \
  --lift-duration "$lift_duration"

if [[ "$plan_only" == true ]]; then
  echo "Plan-only complete; no robot control interface was contacted."
  exit 0
fi

echo
echo "[3/3] Visual grasp sequence: robot=$robot lift=${lift_duration}s/${lift_height}m"
grasp_args=(
  --robot "$robot"
  --approach-trajectory "$trajectory"
  --lift-trajectory "$lift_trajectory"
  --initial-vision "$vision_result"
  --target-class "$target_class"
  --capture-backend "$verification_capture_backend"
  --vision-confidence "$vision_conf"
  --vision-runner "$repo_dir/run_vision.sh"
  --verification-timeout "$verification_timeout"
  --close-target-tolerance "$close_target_tolerance"
  --lifted-target-tolerance "$lifted_target_tolerance"
  --minimum-lift-ratio "$minimum_lift_ratio"
  --maximum-lateral-drift "$maximum_lateral_drift"
  --status "$grasp_status"
)
if [[ -n "$camera_calibration" ]]; then
  grasp_args+=(--camera-calibration "$camera_calibration")
fi
if [[ "$execute" == true ]]; then
  grasp_args+=(--execute --confirm-control-authority)
  if [[ "$assume_yes" != true ]]; then
    echo "Live sequence will move the $side arm and OmniPicker. Type RUN to continue:"
    read -r confirmation
    [[ "$confirmation" == "RUN" ]] || { echo "Execution cancelled."; exit 1; }
  fi
  # shellcheck source=/dev/null
  source "$repo_dir/tools/setup_x2_mc_env.sh"
  ros2 run graspv2 x2_aimdk_hardware grasp "${grasp_args[@]}"
else
  PYTHONPATH="$repo_dir${PYTHONPATH:+:$PYTHONPATH}" \
    /usr/bin/python3 -m graspv2.aimdk_hardware grasp "${grasp_args[@]}"
fi
