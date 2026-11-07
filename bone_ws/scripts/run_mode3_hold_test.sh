#!/usr/bin/env bash
set -Ee -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
SETUP_FILE="${WS_ROOT}/install/setup.bash"

bootstrap_environment() {
  if [[ -r "${ROS_SETUP}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
    set -u
  fi
  if [[ -r "${SETUP_FILE}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${SETUP_FILE}"
    set -u
  fi
}

bootstrap_environment

RUN_HARDWARE=false
HARDWARE_QUALIFIED=false
PASSTHROUGH_ARGS=()

for arg in "$@"; do
  if [[ "${arg}" == "--hardware" || "${arg}" == "--hardware-write" || "${arg}" == "--live" ]]; then
    RUN_HARDWARE=true
  elif [[ "${arg}" == "--hardware-qualified" ]]; then
    HARDWARE_QUALIFIED=true
  elif [[ "${arg}" == "--dry-run" ]]; then
    RUN_HARDWARE=false
  else
    PASSTHROUGH_ARGS+=("${arg}")
  fi
done

if [[ "${RUN_HARDWARE}" == "true" ]]; then
  if [[ "${HARDWARE_QUALIFIED}" != "true" ]]; then
    echo "==================================================================" >&2
    echo "ERROR: --hardware requires explicit --hardware-qualified authorization." >&2
    echo "Safety policy prohibits implicit or automatic qualification." >&2
    echo "To run on real hardware, you must explicitly pass BOTH:" >&2
    echo "  --hardware --hardware-qualified" >&2
    echo "==================================================================" >&2
    exit 2
  fi

  echo "=================================================================="
  echo "6PUS Mode 3 Holding Force Handover Test [REAL HARDWARE MODE]"
  echo "Hardware write interface: ENABLED (Native Mode 3 Constant Hold)"
  echo "Safety Guards Active:"
  echo "  1. Initial Mode 0 Pre-flight Audit (Modes=0, Faults=0, Currents<500mA)"
  echo "  2. Signed Holding Force Handover Baseline (0x2C -> 0x27, Delta F = 0)"
  echo "  3. 500 mA Unified Motor Current Protection"
  echo "  4. 6 steps (0.15 mm) Actuator Displacement Protection"
  echo "  5. 1.0 s Periodic Mode 3 Readback Verification (0x25 == 3)"
  echo "  6. Automatic Mode 0 Position Re-anchoring on Exit/Fault"
  echo "=================================================================="

  exec python3 "${SCRIPT_DIR}/run_mode3_hold_test.py" \
    --hardware \
    --hardware-qualified \
    "${PASSTHROUGH_ARGS[@]}"
else
  echo "=================================================================="
  echo "6PUS Mode 3 Holding Force Handover Test [DRY-RUN READ-ONLY AUDIT]"
  echo "Hardware writes disabled: dry_run=true, hardware_write_enabled=false"
  echo "Performs read-only audit of 6 axes in Mode 0 (0 hardware writes)."
  echo "To enable real hardware Mode 3 execution, pass: --hardware --hardware-qualified"
  echo "=================================================================="

  exec python3 "${SCRIPT_DIR}/run_mode3_hold_test.py" \
    --dry-run \
    "${PASSTHROUGH_ARGS[@]}"
fi
