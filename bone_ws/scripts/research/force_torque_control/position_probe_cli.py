#!/usr/bin/env python3
"""Explicit one-axis real position probe; does not operate the torque controller."""
import argparse
import json
import signal
from pathlib import Path
import time
import sys
from position_probe import PositionProbe, PositionStepTarget
from live_readonly import check_free
from pus6_motor_driver.uart_bus import UartBus


def main(argv=None):
    p=argparse.ArgumentParser(description='One bounded real actuator position command, no homing or automatic return')
    p.add_argument('--execute',action='store_true',help='actually open UART and write actuator registers')
    p.add_argument('--axis',type=int,required=True); p.add_argument('--delta-steps',type=int,required=True)
    p.add_argument('--max-delta-steps',type=int,required=True); p.add_argument('--observe-s',type=float,required=True)
    p.add_argument('--tolerance-steps',type=int,required=True); p.add_argument('--log-dir',required=True)
    args=p.parse_args(argv)
    if not args.execute: p.error('--execute is required for any serial access')
    if not 1<=args.axis<=6 or args.delta_steps==0 or args.max_delta_steps<=0 or abs(args.delta_steps)>args.max_delta_steps:
        p.error('invalid axis or displacement bound')
    if not 0<args.observe_s<=10 or not 0<=args.tolerance_steps<abs(args.delta_steps):
        p.error('invalid observation duration or tolerance')
    path=Path(args.log_dir); path.mkdir(parents=True,exist_ok=False)
    result={'axis':args.axis,'delta_steps':args.delta_steps,'command_attempted':False,'target_reached':False,
            'pause_acknowledged':False,'feedback_stable_after_pause':False,'hardware_stop_verified':False}
    stream=(path/'events.jsonl').open('x',encoding='utf-8',buffering=1)
    def record(item): stream.write(json.dumps(item,allow_nan=False)+'\n')
    bus=None; probe=None
    def terminate(_signal,_frame): raise KeyboardInterrupt('termination requested')
    old_sigterm=signal.signal(signal.SIGTERM,terminate)
    try:
        check_free('/dev/ttyUSB0')
        import serial
        ser=serial.Serial('/dev/ttyUSB0',baudrate=921600,timeout=.05,write_timeout=.05,exclusive=True)
        try: bus=UartBus(serial_instance=ser,timeout=.05,retries=0,read_only=False)
        except Exception: ser.close(); raise
        probe=PositionProbe(bus,args.axis,args.max_delta_steps,record)
        first=probe.preflight(); target=first.current_position+args.delta_steps
        result['initial_steps']=first.current_position; result['target_steps']=target
        probe.command(PositionStepTarget(args.axis,target))
        deadline=time.monotonic()+args.observe_s
        while time.monotonic()<deadline:
            s=probe.read()
            if abs(s.current_position-target)<=args.tolerance_steps:
                result['target_reached']=True; break
            time.sleep(.05)
    except (Exception,KeyboardInterrupt) as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
    finally:
        if probe:
            result['command_attempted']=probe.issued
        if probe and probe.issued:
            try:
                probe.pause_once(); result['pause_acknowledged']=True
            except Exception as exc: result['pause_error']=f'{type(exc).__name__}: {exc}'
            positions=[]
            try:
                for _ in range(3):
                    positions.append(probe.read().current_position); time.sleep(.1)
                result['post_pause_positions_steps']=positions
                result['feedback_stable_after_pause']=max(positions)-min(positions)<=args.tolerance_steps
            except Exception as exc: result['post_pause_error']=f'{type(exc).__name__}: {exc}'
        if bus:
            result['write_audit']=bus.write_audit; bus.close()
        stream.close()
        signal.signal(signal.SIGTERM,old_sigterm)
        (path/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['target_reached'] and result['pause_acknowledged'] and result['feedback_stable_after_pause'] and 'error' not in result else 2
if __name__=='__main__': sys.exit(main())
