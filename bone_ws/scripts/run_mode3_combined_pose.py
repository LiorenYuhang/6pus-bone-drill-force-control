#!/usr/bin/env python3
"""Source-tree entry for simultaneous six-DOF Mode 3 tracking."""
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[1]/'src'
for package in ('pus6_force_control', 'pus6_motor_driver', 'pus6_kinematics', 'pus6_control'):
    sys.path.insert(0, str(SRC/package))

from pus6_force_control.combined_pose_motion import main

if __name__ == '__main__':
    raise SystemExit(main())
