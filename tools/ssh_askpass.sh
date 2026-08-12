#!/usr/bin/env bash
# OpenSSH invokes this helper for non-interactive robot password prompts.
set -euo pipefail

printf '%s\n' "${GRASPV2_ROBOT_PASSWORD:-1}"
