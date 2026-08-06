#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
graspv2_root="$(cd -- "$script_dir/.." && pwd)"
aimdk_setup="${GRASPV2_AIMDK_SETUP:-}"

if [[ -z "$aimdk_setup" ]]; then
  for candidate in \
    "$graspv2_root/aimdk0907_install/setup.bash" \
    "/home/svt/Raicom2026-old/aimdk_msgs/install/setup.bash"; do
    if [[ -f "$candidate" ]]; then
      aimdk_setup="$candidate"
      break
    fi
  done
fi

if [[ -z "$aimdk_setup" ]]; then
  exec "$script_dir/build_aimdk0907_overlay.sh"
fi
if [[ ! -f "$aimdk_setup" ]]; then
  echo "AimDK setup not found: $aimdk_setup" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "$aimdk_setup"
set -u

/usr/bin/python3 - <<'PY'
from aimdk_msgs.msg import (
    HandCommandArray,
    JointCommandArray,
    JointState,
    McActionCommand,
    UpperBodyCommandArray,
)
from aimdk_msgs.srv import SetMcAction, SetMcPresetMotion

joint_fields = set(JointState.get_fields_and_field_types())
request_fields = set(SetMcPresetMotion.Request.get_fields_and_field_types())
expected_joint = {"name", "position", "velocity", "effort", "coil_temp", "motor_temp", "motor_vol"}
expected_request = {"header", "area", "motion", "interrupt", "ani_path"}
expected_joint_command = {"header", "joints"}
expected_hand_command = {
    "header", "left_hand_type", "left_hands", "right_hand_type", "right_hands"
}
joint_command_fields = set(JointCommandArray.get_fields_and_field_types())
hand_command_fields = set(HandCommandArray.get_fields_and_field_types())
upper_body_fields = set(UpperBodyCommandArray.get_fields_and_field_types())
mc_action_command_fields = set(McActionCommand.get_fields_and_field_types())
set_action_fields = set(SetMcAction.Request.get_fields_and_field_types())
expected_upper_body = {
    "header", "source", "hand_sub_mode", "head_pos", "arm_pos", "hand_pos"
}
if (
    joint_fields != expected_joint
    or request_fields != expected_request
    or joint_command_fields != expected_joint_command
    or hand_command_fields != expected_hand_command
    or upper_body_fields != expected_upper_body
    or mc_action_command_fields != {"action", "action_desc"}
    or set_action_fields != {"header", "source", "command"}
):
    raise SystemExit(
        "Installed AimDK messages do not match v0.9.0.7: "
        f"JointState={sorted(joint_fields)}, SetMcPresetMotion={sorted(request_fields)}, "
        f"JointCommandArray={sorted(joint_command_fields)}, "
        f"HandCommandArray={sorted(hand_command_fields)}, "
        f"UpperBodyCommandArray={sorted(upper_body_fields)}, "
        f"McActionCommand={sorted(mc_action_command_fields)}, "
        f"SetMcAction={sorted(set_action_fields)}"
    )
PY

cd "$graspv2_root"
colcon build --symlink-install --packages-select graspv2
echo "Built graspv2 with AimDK overlay: $aimdk_setup"
