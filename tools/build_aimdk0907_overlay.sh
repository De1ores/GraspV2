#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
graspv2_root="$(cd -- "${script_dir}/.." && pwd)"
interface_source="${graspv2_root}/vendor/aimdk_msgs_0907"

if [[ ! -f "${interface_source}/package.xml" ]]; then
  echo "Bundled AimDK v0.9.0.7 interface package is missing" >&2
  exit 1
fi

# A clean environment prevents Conda Python from being mixed with the system
# ROS 2 Python while rosidl generates the bindings.
exec env -i \
  PATH=/usr/bin:/bin \
  GRASPV2_ROOT="${graspv2_root}" \
  INTERFACE_SOURCE="${interface_source}" \
  /bin/bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    set -u
    colcon --log-base "${GRASPV2_ROOT}/aimdk0907_log" build \
      --base-paths "${INTERFACE_SOURCE}" \
      --build-base "${GRASPV2_ROOT}/aimdk0907_build" \
      --install-base "${GRASPV2_ROOT}/aimdk0907_install" \
      --packages-select aimdk_msgs \
      --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
    set +u
    source "${GRASPV2_ROOT}/aimdk0907_install/setup.bash"
    set -u
    cd "${GRASPV2_ROOT}"
    colcon build --symlink-install --packages-select graspv2
  '
