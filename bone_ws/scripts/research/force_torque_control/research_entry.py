#!/usr/bin/env python3
"""Unified synthetic/replay research entry. No physical transport factory."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import uuid
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from imu_node import ImuResearchNode
from offline_node import OfflineControlNode
from status_node import ForceStatusNode
from mock_inputs import MockInputs
from run_log import RunLog
from core import Mode

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]

def make_parser():
    p=argparse.ArgumentParser(description='Mock-only 6PUS research runtime')
    p.add_argument('--config',required=True)
    p.add_argument('--mode',choices=['PID_TORQUE','ACTUATOR_FORCE_CONTROL','DS_SCTDE'])
    p.add_argument('--input',choices=['synthetic','external'])
    p.add_argument('--status-source',choices=['mock','hardware'],help='provenance required for external raw status; never opens a port')
    p.add_argument('--duration',type=float,default=0.,help='seconds; 0 runs until Ctrl+C')
    p.add_argument('--log-dir',help='new directory; default results/research_control/session-...')
    return p

def main(argv=None):
    args=make_parser().parse_args(argv)
    with open(args.config,encoding='utf-8') as stream: c=json.load(stream)
    if c.get('backend') != 'mock': raise ValueError('real backend unavailable; select mock')
    mode=args.mode or c['mode']; source=args.input or c.get('input_mode','external')
    status_source=args.status_source or c.get('status_source','mock')
    if mode not in ('PID_TORQUE','ACTUATOR_FORCE_CONTROL','DS_SCTDE'): raise ValueError('mode unavailable')
    if source not in ('synthetic','external'): raise ValueError('invalid input_mode')
    if status_source not in ('mock','hardware') or (source=='synthetic' and status_source!='mock'): raise ValueError('invalid status provenance')
    confirmed=c.get('mock_gf_assumption') is True if status_source=='mock' else c.get('force_unit_confirmed') is True
    if not math.isfinite(args.duration) or args.duration<0: raise ValueError('invalid duration')
    if mode=='ACTUATOR_FORCE_CONTROL' and not confirmed:
        raise ValueError('force g unit conversion must be explicitly confirmed for this source')
    run_dir=Path(args.log_dir).expanduser() if args.log_dir else REPO/'results'/'research_control'/('session-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    run_dir.mkdir(parents=True,exist_ok=False)
    (run_dir/'config.json').write_text(json.dumps({'config':c,'mode':mode,'input_mode':source,'status_source':status_source,'hardware_write_count':0},indent=2)+'\n')
    print(f'LOG_DIR={run_dir}',flush=True)
    if mode=='DS_SCTDE':
        print('DS_SCTDE uses the existing standalone offline closed-loop simulation, not the ROS actuator chain.',flush=True)
        return subprocess.call([sys.executable,str(HERE.parent/'ds_sctde_closed_loop'/'simulate.py'),'--output',str(run_dir/'ds_sctde')])
    rclpy.init(); nodes=[]; executor=None; logger=None; control=None; reason='normal'; code=0
    try:
        control=OfflineControlNode([Parameter('enabled',value=True),Parameter('mode',value=mode),
            Parameter('config_json',value=json.dumps(c['control'])),
            Parameter('target_max_age_s',value=float(c.get('target_max_age_s',.2)))])
        nodes.append(control)
        imu=ImuResearchNode([Parameter('enabled',value=True),Parameter('config_json',value=json.dumps(c['imu'])),
            Parameter('imu_max_age_s',value=float(c.get('imu_max_age_s',.2)))])
        nodes.append(imu)
        status=ForceStatusNode([Parameter('source',value=status_source),
            Parameter('gram_force_confirmed',value=confirmed),
            Parameter('max_age_s',value=float(c['control']['force']['max_age_s'])),
            Parameter('publish_position',value=True)])
        nodes.append(status)
        logger=RunLog(run_dir/'telemetry.jsonl'); nodes.append(logger)
        if source=='synthetic':
            mock=MockInputs(control.session.force.bus); nodes.append(mock)
        executor=SingleThreadedExecutor()
        for node in nodes: executor.add_node(node)
        logger.record('start',{'mode':mode,'input_mode':source,'status_source':status_source,'backend':'mock','hardware_write_count':0})
        print(f'START mode={mode} input={source} backend=mock hardware_write_count=0',flush=True)
        start=time.monotonic()
        while rclpy.ok():
            executor.spin_once(timeout_sec=.05)
            if imu.fault or control.session.mode is Mode.FAULT:
                reason='imu: '+imu.fault if imu.fault else 'control: '+str(control.session.reason)
                code=2; break
            if args.duration and time.monotonic()-start>=args.duration: break
        if code==0 and source=='synthetic' and args.duration and not (logger.counts['force'] if mode=='ACTUATOR_FORCE_CONTROL' else logger.counts['torque']):
            reason='no mock control command observed'; code=2
    except KeyboardInterrupt:
        reason='interrupted'
    except Exception as exc:
        reason=str(exc); code=2
    finally:
        if control is not None and control.session is not None:
            try: control.session.stop()
            except Exception as exc: reason='mock stop failed: '+str(exc); code=2
        if logger is not None:
            logger.record('stop',{'reason':reason,'mode':control.session.mode.value if control and control.session else 'unavailable',
                                 'counts':logger.counts,'hardware_write_count':0})
            logger.close()
        if executor is not None: executor.shutdown()
        for node in nodes: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    print(f'STOP reason={reason} hardware_write_count=0',flush=True)
    return code
if __name__=='__main__': sys.exit(main())
