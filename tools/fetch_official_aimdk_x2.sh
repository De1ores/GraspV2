#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
version="1.0.0"
revision="a424add7"
archive_name="aimdk-aarch64-${revision}-artifacts.zip"
archive_sha256="5bbcf724d54fb28f153db0d272f9acb7906bb1d2cac7dd7ccdc699a5c7eeab35"
download_url="https://x2-aimdk.agibot.com/downloads/${archive_name}"
runtime_dir="$repo_dir/offline/runtime/aimdk-x2-v${version}"
archive="$runtime_dir/$archive_name"
sdk_root="$runtime_dir/aimdk-aarch64-${revision}-artifacts"
download_only=false

if [[ "${1:-}" == "--download-only" ]]; then
  download_only=true
  shift
fi
if (($#)); then
  echo "Usage: $0 [--download-only]" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash && "$download_only" != true ]]; then
  echo "ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
for command in curl sha256sum unzip; do
  command -v "$command" >/dev/null || {
    echo "Required command is missing: $command" >&2
    exit 1
  }
done

mkdir -p "$runtime_dir"
if [[ -f "$archive" ]]; then
  printf '%s  %s\n' "$archive_sha256" "$archive" | sha256sum --check --status || {
    echo "Existing AimDK archive has the wrong SHA-256: $archive" >&2
    echo "Move it aside and run this command again." >&2
    exit 1
  }
else
  partial="$archive.part"
  if [[ -e "$partial" ]]; then
    echo "Incomplete download already exists: $partial" >&2
    echo "Move it aside and run this command again." >&2
    exit 1
  fi
  echo "Downloading official X2 AimDK v${version}..."
  curl --fail --location --retry 3 --output "$partial" "$download_url"
  printf '%s  %s\n' "$archive_sha256" "$partial" | sha256sum --check --status || {
    echo "Downloaded AimDK archive failed SHA-256 verification: $partial" >&2
    exit 1
  }
  mv -- "$partial" "$archive"
fi
unzip -tq "$archive" >/dev/null
echo "Verified official archive: $archive"

if [[ ! -d "$sdk_root" ]]; then
  extract_dir="$(mktemp -d "$runtime_dir/.extract.XXXXXXXX")"
  cleanup_extract() {
    rm -rf -- "$extract_dir"
  }
  trap cleanup_extract EXIT
  unzip -q "$archive" -d "$extract_dir"
  extracted="$extract_dir/aimdk-aarch64-${revision}-artifacts"
  [[ -f "$extracted/version" ]] || {
    echo "Official archive did not contain the expected SDK root" >&2
    exit 1
  }
  mv -- "$extracted" "$sdk_root"
  trap - EXIT
  rmdir -- "$extract_dir"
fi
grep -Fx "aimdk v${version}-g${revision%?}" "$sdk_root/version" >/dev/null || {
  echo "Unexpected SDK version file: $sdk_root/version" >&2
  exit 1
}

if [[ "$download_only" == true ]]; then
  echo "AimDK v${version} download and extraction complete: $sdk_root"
  exit 0
fi

if [[ "$(uname -m)" == "aarch64" ]]; then
  suffix=""
else
  suffix="_host"
fi
setup="$sdk_root/install${suffix}/setup.bash"
if [[ ! -f "$setup" ]]; then
  echo "Building the official aimdk_msgs interfaces for $(uname -m)..."
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  set -u
  (
    cd "$sdk_root"
    PATH=/usr/bin:/bin:/usr/sbin:/sbin \
      PYTHONNOUSERSITE=1 \
      /usr/bin/colcon --log-base "log${suffix}" build \
        --packages-select aimdk_msgs \
        --build-base "build${suffix}" \
        --install-base "install${suffix}" \
        --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
  )
fi

set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "$setup"
set -u
PYTHONNOUSERSITE=1 /usr/bin/python3 \
  "$script_dir/check_aimdk_overlay.py" --require-capability animation
echo "Official X2 AimDK v${version} is ready: $setup"
