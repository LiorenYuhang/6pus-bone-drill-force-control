#!/usr/bin/env bash
set -Ee -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
if [[ -r "${SCRIPT_DIR}/../install/setup.bash" ]]; then
  source "${SCRIPT_DIR}/../install/setup.bash"
fi
set -u
exec python3 -u "${SCRIPT_DIR}/run_mode3_combined_pose.py" "$@"
