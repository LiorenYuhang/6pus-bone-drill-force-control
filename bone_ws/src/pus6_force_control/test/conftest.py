"""Support source-tree pytest as well as colcon's installed test environment."""
from pathlib import Path
import sys

src = Path(__file__).resolve().parents[2]
for name in ("pus6_force_control", "pus6_motor_driver", "pus6_kinematics"):
    sys.path.insert(0, str(src / name))
