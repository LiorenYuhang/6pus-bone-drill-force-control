#!/usr/bin/env python3
"""Run the three research-only ROS nodes in one executor; no hardware factory."""
import argparse
import json
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from imu_node import ImuResearchNode
from offline_node import OfflineControlNode
from status_node import ForceStatusNode

def main():
    p=argparse.ArgumentParser(); p.add_argument('--config',required=True)
    args=p.parse_args()
    with open(args.config,encoding='utf-8') as stream: c=json.load(stream)
    if c.get('backend') != 'mock': raise ValueError('only mock backend implemented')
    rclpy.init(); nodes=[]
    try:
        control=OfflineControlNode([Parameter('enabled',value=True),Parameter('mode',value=c['mode']),
            Parameter('config_json',value=json.dumps(c['control'])),
            Parameter('target_max_age_s',value=float(c.get('target_max_age_s',.2)))])
        nodes.append(control)
        imu=ImuResearchNode([Parameter('enabled',value=True),Parameter('config_json',value=json.dumps(c['imu'])),
            Parameter('imu_max_age_s',value=float(c.get('imu_max_age_s',.2)))])
        nodes.append(imu)
        status=ForceStatusNode([Parameter('source',value='mock'),
            Parameter('gram_force_confirmed',value=bool(c.get('mock_gf_assumption',False)))])
        nodes.append(status)
        executor=SingleThreadedExecutor()
        for node in nodes: executor.add_node(node)
        executor.spin()
    finally:
        for node in nodes: node.destroy_node()
        rclpy.shutdown()
if __name__=='__main__': main()
