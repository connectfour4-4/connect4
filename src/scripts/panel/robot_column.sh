#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 1 ]]; then
  echo "Usage: $0 <column>"
  exit 2
fi

column="$1"

case "${column}" in
  1|4|7)
    ;;
  *)
    echo "Expected column 1, 4, or 7, got: ${column}"
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_DIR="$(cd "${SRC_DIR}/.." && pwd)"

source "${WORKSPACE_DIR}/devel/setup.bash"

rostopic pub -1 /robot_next_move std_msgs/Int8 "data: ${column}"
