"""Existing J14 feedback bridge plus asynchronous FK/experiment recording."""
import argparse
import json
import os
import signal
import socket
import time
import numpy as np
import rclpy

from pus6_motor_driver.j14_feedback_bridge import Pus6FeedbackBridgeNode
from pus6_motor_driver.j14_process_uart import send_packet
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from .force_pipeline import raw_steps_to_slider_m
from .z_position_motion import hold_sample_settled
from .mode3_dual_imu import DualImuMotionConfig
from .process_recording import AsyncRecords


class MotionFeedbackBridge(Pus6FeedbackBridgeNode):
    def __init__(self,sock,config):
        super().__init__(sock,config)
        self.kinematics=Analytical6PusKinematics();self.guess=None
        self.settings=DualImuMotionConfig(**config['motion_settings'])
        self.stable_since=None;self.fk_status=None
        self.records=AsyncRecords(self.out_dir,transform=self.with_fk)

    def with_fk(self,name,row):
        q=raw_steps_to_slider_m(row['actual_positions_steps'],self.kinematics.geometry_config)
        fk=self.kinematics.forward_kinematics_with_metrics(q,initial_guess=self.guess)
        pose=list(fk.pose) if fk.converged and np.isfinite(fk.pose).all() else None
        if pose is not None:self.guess=pose
        row=dict(row,actual_pose_m_rad=pose,fk_converged=bool(fk.converged),
            actual_z_m=pose[2] if pose else None,target_z_m=row['target_pose_m_rad'][2])
        settled=hold_sample_settled(row['position_error_steps'],row['pid']['joint_velocity_m_s'],
            row['actual_z_m'],row['target_z_m'],self.settings)
        if row['phase']=='final_hold' and settled:
            if self.stable_since is None:self.stable_since=row['ns']
        else:self.stable_since=None
        self.fk_status=dict(phase=row['phase'],ns=row['ns'],uart_cycle_sequence=row['uart_cycle_sequence'],
            actual_pose_m_rad=pose,settled=settled,hold_complete=self.stable_since is not None and
            (row['ns']-self.stable_since)*1e-9>=self.settings.settle_seconds+self.settings.hold_seconds)
        return row

    def _process_feedback_packet(self,row,*times):
        motion=row.get('motion')
        if motion is not None:
            super()._process_feedback_packet(row,*times)
            self.records.submit('tracking.jsonl',motion)
        status=dict(type='motion_status',epoch=self.config['epoch'],bridge_pid=os.getpid(),
            bridge_ns=time.monotonic_ns(),mode3_start_ns=row.get('mode3_start_ns'),
            reference_ready_ns=row.get('reference_ready_ns'),command_seq=row.get('command_seq'),
            command_publish_ns=row.get('command_publish_ns'),command_receive_ns=row.get('command_receive_ns'),
            uart_cycle_sequence=row['cycle_seq'],actual_positions_steps=row['positions'],
            phase=motion['phase'] if motion else 'waiting',fk=self.fk_status,
            recording_dropped=self.records.dropped,recording_error=self.records.error)
        if motion is not None:
            status.update(target_pose_m_rad=motion['target_pose_m_rad'],
                max_error_steps=max(map(abs,motion['position_error_steps'])),max_current_ma=max(map(abs,motion['currents_ma'])))
        send_packet(self.fb_sock,status)


def main():
    p=argparse.ArgumentParser();p.add_argument('--fd',type=int,required=True);p.add_argument('--config',required=True)
    args=p.parse_args();config=json.loads(args.config)
    sock=socket.socket(fileno=args.fd);sock.setblocking(False)
    if config['hardware_mode']=='mock':os.environ['ROS_DOMAIN_ID']=str(config.get('mock_ros_domain_id',223))
    rclpy.init();node=MotionFeedbackBridge(sock,config)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:node.stop())
    try:node.run_bridge_loop()
    finally:
        node.stop();health=node.records.close()
        (node.out_dir/'motion_bridge_report.json').write_text(json.dumps(health,indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        sock.close()


if __name__=='__main__':main()
