#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
graspv2_root="$(cd -- "$script_dir/.." && pwd)"
aimdk_setup="$("$script_dir/select_aimdk_setup.sh")"

set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "$aimdk_setup"
set -u

PYTHONNOUSERSITE=1 /usr/bin/python3 "$script_dir/check_aimdk_overlay.py"

cd "$graspv2_root"
colcon build --symlink-install --packages-select graspv2
echo "Built graspv2 with AimDK overlay: $aimdk_setup"
