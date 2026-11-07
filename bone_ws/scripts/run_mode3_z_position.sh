#!/usr/bin/env bash
set -Ee -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Source only the existing Force workspace, never the Position/Torque overlay.
set +u
source /opt/ros/jazzy/setup.bash
if [[ -r "${SCRIPT_DIR}/../install/setup.bash" ]]; then
  source "${SCRIPT_DIR}/../install/setup.bash"
fi
set -u
exec python3 "${SCRIPT_DIR}/run_mode3_z_position.py" "$@"
