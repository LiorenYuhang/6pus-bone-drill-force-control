# 6PUS Bone Drill Torque Control

This private research project investigates joint-level motor torque control for the same 6PUS parallel-robot bone drill active stabilization application as [6PUS Bone Drill Position Control](https://github.com/LiorenYuhang/6pus-bone-drill-position-control). The inherited position-control, IMU feedforward, P/PD, LADRC, inverse kinematics, trajectory shaping and position-servo implementation remain a comparison baseline.

The research PID computes six motor torque values in N·m from six actuator-length position errors in metres, using independently configured P, I and D gains. Its derivative is a filtered estimate, not measured velocity. This PID is an engineering experiment, not the DS-SCTDE controller in Zhang et al.; the paper's equation (36) is the APD comparison controller.

**Status: offline only.** The local actuator documentation does not establish the motor-shaft torque command interface. No real torque command is encoded, transmitted or enabled; the documented actuator force-control target is not a motor torque interface. The experiment is not hardware qualified.

`bone_ws/scripts/research/joint_torque_pid/` contains the independent PID, a ROS 2 diagnostic-only node and a strictly in-memory mock pipeline. When explicitly enabled, its synthetic feedback callback drives that mock pipeline. The mock's six-float format is a test fixture, not a motor protocol. Its mode exclusion is local to one mock pipeline and does not protect shared physical hardware. It cannot replace a sole serial owner, verified interprocess mode transition, or real device stop. A timeout or fault in the mock does not stop a motor. See [the research report](docs/research/6PUS_JOINT_TORQUE_PID_RESEARCH.md).

## Layout and offline tests

- `bone_ws/src/`: inherited ROS 2 kinematics, control, driver, messages and IMU packages.
- `bone_ws/scripts/research/joint_torque_pid/`: torque research and software tests.
- `docs/`: user-facing documentation; `results/`: inherited experimental evidence.

With Ubuntu and ROS 2 Jazzy, build into an isolated directory before running ROS tests:

```bash
cd bone_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/6pus-research-log build --base-paths src --packages-up-to pus6_control pus6_motor_driver --build-base /tmp/6pus-research-build --install-base /tmp/6pus-research-install
source /tmp/6pus-research-install/setup.bash
python3 -m pytest -q scripts/research/joint_torque_pid/test_controller.py scripts/research/joint_torque_pid/test_offline_ros_node.py scripts/research/joint_torque_pid/test_mock_pipeline.py
```

These tests use no physical motor. Do not use the copied `bone_ws/install` overlay for research: it can still refer to the original workspace. Real hardware operation requires the verified manufacturer torque interface, axis mapping and tuning, exclusive device ownership, validated stop behavior, and explicit human authorization.

The repository has no established repository-wide license. ROS manifest license metadata does not establish rights for all materials; third-party components retain their notices and require separate review.
