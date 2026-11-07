#!/usr/bin/env python3
"""Run the measured-start Mode 3 single-axis experiments."""
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[1]/'src'
for package in ('pus6_force_control', 'pus6_motor_driver', 'pus6_kinematics', 'pus6_control'):
    sys.path.insert(0, str(SRC/package))

from pus6_force_control.single_axis_motion import main

if __name__ == '__main__':
    raise SystemExit(main())
