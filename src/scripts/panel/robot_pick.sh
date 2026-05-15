#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_DIR="$(cd "${SRC_DIR}/.." && pwd)"

source "${WORKSPACE_DIR}/devel/setup.bash"

exec rosrun move panel_pick.py
