from glob import glob
from setuptools import find_packages, setup

setup(
    name="pus6_force_control",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/pus6_force_control"]),
        ("share/pus6_force_control", ["package.xml"]),
        ("share/pus6_force_control/config", glob("config/*.yaml") + glob("config/*.json")),
        ("share/pus6_force_control/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML", "scipy"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="bone_drill",
    maintainer_email="dev@example.com",
    description="6PUS task-space force control foundations (offline stage 1)",
    license="MIT",
    entry_points={"console_scripts": [
        "force_assisted_home = pus6_force_control.assisted_home:main",
        "force_manual_recovery = pus6_force_control.manual_recovery:main",
        "force_pid_offline = pus6_force_control.offline_demo:main",
        "pus6_force_control_node = pus6_force_control.force_control_node:main",
        "force_active_stabilization = pus6_force_control.active_stabilization_node:main",
        "stage4_hardware_experiment = pus6_force_control.stage4_experiment:main",
        "mode3_hold_test = pus6_force_control.mode3_hold_experiment:main",
        "mode3_z_motion = pus6_force_control.mode3_z_motion_experiment:main",
        "mode3_z_position = pus6_force_control.z_position_motion:main",
        "mode3_single_axis = pus6_force_control.single_axis_motion:main",
        "mode3_combined_pose = pus6_force_control.combined_pose_motion:main",
        "mode3_dual_imu = pus6_force_control.mode3_dual_imu:main",
    ]},
)
