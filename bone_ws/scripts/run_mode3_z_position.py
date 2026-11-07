#!/usr/bin/env python3
"""Source-tree entry for native Mode 3 position-feedback Z motion."""
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[1] / 'src'
for package in ('pus6_force_control', 'pus6_motor_driver', 'pus6_kinematics', 'pus6_control'):
    sys.path.insert(0, str(SRC / package))

from pus6_force_control.z_position_motion import main

if __name__ == '__main__':
    raise SystemExit(main())
