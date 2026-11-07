#!/usr/bin/env python3
"""Force-only source-tree entry; default is synthetic IMU + Mock bus."""
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[1]/'src'
for name in ('pus6_force_control', 'pus6_motor_driver', 'pus6_kinematics', 'pus6_control'):
    sys.path.insert(0, str(SRC/name))

from pus6_force_control.mode3_dual_imu import main

if __name__ == '__main__':
    raise SystemExit(main())
