#!/usr/bin/env bash
set -Ee -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "${SCRIPT_DIR}/../install/setup.bash"
set -u
exec python3 -u "${SCRIPT_DIR}/run_mode3_dual_imu.py" "$@"
