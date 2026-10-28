"""Read-only availability decision for the current J14 research path.

No ROS, serial, numerical torque or command access. In particular a ROS field's
existence is not proof that its value was populated or is physically calibrated.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Availability:
    length_and_host_rx_time: bool = True
    length_velocity_validated: bool = False
    attitude_acceleration_validated: bool = False
    task_wrench_validated_and_aligned: bool = False
    motor_torque_interface_verified: bool = False


def audit(a=Availability()):
    reasons=[]
    if not a.length_and_host_rx_time: reasons.append('missing length or host receive time')
    if not a.length_velocity_validated: reasons.append('no validated causal six-axis velocity')
    if not a.attitude_acceleration_validated: reasons.append('no aligned validated task angular acceleration')
    if not a.task_wrench_validated_and_aligned: reasons.append('no valid aligned applied task wrench')
    if not a.motor_torque_interface_verified: reasons.append('motor torque interface not verified')
    return {'ready':not reasons, 'reasons':reasons, 'torque_command':None, 'hardware_write_count':0}
