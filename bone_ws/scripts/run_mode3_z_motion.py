#!/usr/bin/env python3
"""Run 6PUS Mode 3 Dynamic Force Control and Z-Axis Motion Experiment.

Applies task-space Z-axis dynamic force commands (+/- amplitude N) via the
existing Jacobian force allocation in native Mode 3, observes mobile platform
upward/downward Z-axis motion via FK and actual force feedback, records all
axis targets, feedback, positions, currents, and task-space motions, and cleanly
exits to Mode 0 with position re-anchoring.
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

from pus6_force_control.mode3_z_motion_experiment import main

if __name__ == '__main__':
    raise SystemExit(main())
