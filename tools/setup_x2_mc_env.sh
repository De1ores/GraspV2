#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Run this file with: source tools/setup_x2_mc_env.sh" >&2
  exit 1
fi

graspv2_env_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

graspv2_aimdk_setup="${GRASPV2_AIMDK_SETUP:-}"
if [[ -z "${graspv2_aimdk_setup}" ]]; then
  for graspv2_aimdk_candidate in \
    "${graspv2_env_root}/aimdk0907_install/setup.bash" \
    "/home/svt/Raicom2026-old/aimdk_msgs/install/setup.bash"; do
    if [[ -f "${graspv2_aimdk_candidate}" ]]; then
      graspv2_aimdk_setup="${graspv2_aimdk_candidate}"
      break
    fi
  done
fi
if [[ -z "${graspv2_aimdk_setup}" || ! -f "${graspv2_aimdk_setup}" ]]; then
  echo "Missing matching AimDK overlay; run tools/build_graspv2_with_installed_aimdk.sh" >&2
  return 1
fi
if [[ ! -f "${graspv2_env_root}/install/local_setup.bash" ]]; then
  echo "Missing graspv2 install; run tools/build_graspv2_with_installed_aimdk.sh" >&2
  return 1
fi

graspv2_env_restore_nounset=false
if [[ "$-" == *u* ]]; then
  graspv2_env_restore_nounset=true
  set +u
fi

source /opt/ros/humble/setup.bash
source "${graspv2_aimdk_setup}"
source "${graspv2_env_root}/install/local_setup.bash"

if [[ "${graspv2_env_restore_nounset}" == true ]]; then
  set -u
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
if [[ -n "${GRASPV2_FASTDDS_PROFILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${GRASPV2_FASTDDS_PROFILE}"
  graspv2_transport="custom Fast DDS profile ${FASTRTPS_DEFAULT_PROFILES_FILE}"
elif ip -o -4 addr show 2>/dev/null | grep -q '10\.0\.200\.2/'; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${graspv2_env_root}/config/fastdds_wired.xml"
  graspv2_transport="wired interface 10.0.200.2"
else
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  graspv2_transport="system DDS transport"
fi

echo "X2 MC environment ready with ${graspv2_transport}"
echo "AimDK overlay: ${graspv2_aimdk_setup}"
