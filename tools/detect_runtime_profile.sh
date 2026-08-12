#!/usr/bin/env bash
# Resolve whether this process runs on the competition robot or a test setup.
set -euo pipefail

requested="${GRASPV2_RUNTIME_PROFILE:-auto}"
case "$requested" in
  competition|test)
    printf '%s\n' "$requested"
    exit 0
    ;;
  auto) ;;
  *)
    echo "GRASPV2_RUNTIME_PROFILE must be auto, competition, or test" >&2
    exit 2
    ;;
esac

# The verified competition image runs locally as agi on aarch64 and owns the
# system AimDK install.  Requiring all three avoids treating an SVT Orin/test
# workstation as the competition robot merely because both use aarch64.
if [[ "$(uname -m)" == "aarch64" ]] &&
   [[ "$(id -un)" == "agi" ]] &&
   [[ -f /home/agi/aimdk/install/setup.bash ]]; then
  printf '%s\n' competition
else
  printf '%s\n' test
fi
