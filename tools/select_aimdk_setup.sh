#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
probe_script="$script_dir/check_aimdk_overlay.py"
explicit_setup="${GRASPV2_AIMDK_SETUP:-}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

candidates=()
if [[ -n "$explicit_setup" ]]; then
  candidates+=("$explicit_setup")
else
  if [[ -n "${HOME:-}" ]]; then
    candidates+=("$HOME/aimdk/install/setup.bash")
  fi
  candidates+=(
    "/home/agi/aimdk/install/setup.bash"
    "/home/svt/Raicom2026-old/aimdk_msgs/install/setup.bash"
    "/home/Raicom2026/aimdk_msgs/install_host/setup.bash"
    "/home/Raicom2026/aimdk_msgs/install/setup.bash"
  )
fi

probe_home="${HOME:-/tmp}"
failures=()
checked=()
declare -A seen=()
for candidate in "${candidates[@]}"; do
  if [[ -n "${seen[$candidate]:-}" ]]; then
    continue
  fi
  seen["$candidate"]=1
  if [[ ! -f "$candidate" ]]; then
    if [[ -n "$explicit_setup" ]]; then
      echo "GRASPV2_AIMDK_SETUP does not exist: $candidate" >&2
      exit 1
    fi
    continue
  fi
  checked+=("$candidate")
  if probe_output="$(
    env -i \
      HOME="$probe_home" \
      PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
      LANG="${LANG:-C.UTF-8}" \
      /bin/bash --noprofile --norc -c '
        set +u
        source /opt/ros/humble/setup.bash >/dev/null
        source "$1" >/dev/null
        PYTHONNOUSERSITE=1 /usr/bin/python3 "$2"
      ' graspv2-aimdk-probe "$candidate" "$probe_script" 2>&1
  )"; then
    printf '%s\n' "$candidate"
    exit 0
  fi
  probe_output="${probe_output//$'\n'/; }"
  failures+=("$candidate: $probe_output")
done

if [[ ${#checked[@]} -eq 0 ]]; then
  echo "No installed AimDK setup.bash was found" >&2
else
  echo "No installed AimDK overlay matches the GraspV2 control schema:" >&2
  printf '  - %s\n' "${failures[@]}" >&2
fi
echo "Set GRASPV2_AIMDK_SETUP to a compatible installed SDK overlay." >&2
exit 1
