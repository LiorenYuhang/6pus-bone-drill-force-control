#!/usr/bin/env python3
"""Run 6PUS Mode 3 Holding Force Handover Test.

Captures actual signed holding force (0x2C), enters native Mode 3 with constant
0x27 targets equal to the captured holding force (no PID correction, Delta F = 0),
maintains force control for a short bounded duration (default 3.0s), logs all
axis modes, targets, forces, positions, and currents, and cleanly exits to Mode 0
with measured position re-anchoring.
"""
import sys
from pathlib import Path

# Add src directories to sys.path if not running from installed workspace
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
for pkg in ('pus6_force_control', 'pus6_motor_driver', 'pus6_kinematics'):
    pkg_path = REPO_ROOT / 'bone_ws' / 'src' / pkg
    if pkg_path.exists() and str(pkg_path) not in sys.path:
        sys.path.insert(0, str(pkg_path))

from pus6_force_control.mode3_hold_experiment import main

if __name__ == '__main__':
    raise SystemExit(main())
