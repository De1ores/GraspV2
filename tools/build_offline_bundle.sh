#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
bundle_name="${GRASPV2_BUNDLE_NAME:-graspv2-x2-ultra-offline}"
dist_dir="${GRASPV2_DIST_DIR:-$repo_dir/dist}"
stage_parent="$(mktemp -d)"
stage="$stage_parent/$bundle_name"
trap 'rm -rf -- "$stage_parent"' EXIT

for required in \
  "$repo_dir/offline/wheelhouse/aarch64/planning" \
  "$repo_dir/offline/wheelhouse/aarch64/vision" \
  "$repo_dir/offline/runtime/libcusparse_lt-linux-aarch64-0.7.1.0-archive.tar.xz" \
  "$repo_dir/mobileclip2_b.ts" \
  "$repo_dir/yoloe-26s-seg.pt"; do
  [[ -e "$required" ]] || { echo "Missing offline asset: $required" >&2; exit 1; }
done

mkdir -p "$stage" "$dist_dir"
tar -C "$repo_dir" \
  --exclude=.git \
  --exclude=.venv \
  --exclude=.planning-venv \
  --exclude=.vision-venv \
  --exclude=.runtime \
  --exclude=.pytest_cache \
  --exclude=__pycache__ \
  --exclude='*/__pycache__' \
  --exclude=build \
  --exclude=install \
  --exclude=log \
  --exclude=dist \
  --exclude=output \
  --exclude=offline/MANIFEST.sha256 \
  -cf - . | tar -C "$stage" -xf -

chmod +x "$stage/install_offline.sh" "$stage/offline_run.sh" \
  "$stage/run.sh" "$stage/run_vision.sh" "$stage/run_full_grasp_pipeline.sh"

(
  cd "$stage"
  find . -type f ! -path './offline/MANIFEST.sha256' -print0 \
    | sort -z \
    | xargs -0 sha256sum > offline/MANIFEST.sha256
)

archive="$dist_dir/${bundle_name}.tar.gz"
tar -C "$stage_parent" -czf "$archive" "$bundle_name"
(cd "$dist_dir" && sha256sum "$(basename "$archive")") > "$archive.sha256"
echo "$archive"
du -h "$archive"
