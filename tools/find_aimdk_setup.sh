#!/usr/bin/env bash
set -euo pipefail

# Search common X2/AimDK layouts, then verify imports instead of trusting a
# directory name. The explicit variable always wins for firmware pinning.
explicit_setup="${GRASPV2_AIMDK_SETUP:-}"
candidates=()
if [[ -n "$explicit_setup" ]]; then
  candidates+=("$explicit_setup")
else
  candidates+=(
    "/home/agi/aimdk/install/setup.bash"
    "/home/agi/AimDK/install/setup.bash"
    "/home/agi/AimDk/install/setup.bash"
    "/opt/aimdk/install/setup.bash"
    "/opt/AimDK/install/setup.bash"
  )
  search_root="${GRASPV2_AIMDK_SEARCH_ROOT:-/home/agi}"
  if [[ -d "$search_root" ]]; then
    while IFS= read -r discovered; do
      candidates+=("$discovered")
    done < <(
      find "$search_root" -maxdepth 7 -type f \
        -ipath '*/aimdk*/install/setup.bash' \
        -print 2>/dev/null | sort
    )
  fi
fi

declare -A seen=()
failures=()
for candidate in "${candidates[@]}"; do
  [[ -n "$candidate" && -z "${seen[$candidate]:-}" ]] || continue
  seen["$candidate"]=1
  [[ -f "$candidate" ]] || continue
  if /bin/bash --noprofile --norc -c '
    set +u
    source /opt/ros/humble/setup.bash >/dev/null
    source "$1" >/dev/null
    PYTHONNOUSERSITE=1 /usr/bin/python3 -c \
      "import rclpy, sensor_msgs.msg, aimdk_msgs.msg, aimdk_msgs.srv"
  ' graspv2-aimdk-probe "$candidate" >/dev/null 2>&1; then
    printf '%s\n' "$candidate"
    exit 0
  fi
  failures+=("$candidate")
done

if [[ -n "$explicit_setup" ]]; then
  echo "GRASPV2_AIMDK_SETUP is missing or incompatible: $explicit_setup" >&2
elif ((${#failures[@]})); then
  echo "Found AimDK candidates, but none supplied compatible Python interfaces:" >&2
  printf '  - %s\n' "${failures[@]}" >&2
else
  echo "No AimDK install/setup.bash was found under standard paths or /home/agi" >&2
fi
echo "Set GRASPV2_AIMDK_SETUP to the firmware-matched overlay." >&2
exit 1
