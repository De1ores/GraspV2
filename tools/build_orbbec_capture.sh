#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$repo_dir/build"

g++ -std=c++17 -O2 -Wall -Wextra \
  "$repo_dir/tools/orbbec_capture.cpp" \
  -o "$repo_dir/build/orbbec_capture" \
  $(pkg-config --cflags --libs opencv4) \
  -I/usr/local/include \
  -L/usr/local/lib -lOrbbecSDK \
  -Wl,-rpath,/usr/local/lib

echo "Built $repo_dir/build/orbbec_capture"
