"""J14 UART Child adaptation: measured-position PI-D, no ROS/FK in the loop."""
from dataclasses import asdict
import json
import time
import numpy as np

from pus6_motor_driver.j14_process_uart import ServoCore, CommandState
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.force_units import g_to_n, NOMINAL_N_PER_G
from pus6_kinematics.geometry_config import load_geometry_config
from .joint_position_force_pid import JointPositionForcePID, JointForcePIDConfig
from .force_pipeline import raw_steps_to_slider_m
from .dual_imu_force_policy import recover_holding_bias
from .mode3_process_policy import force_limits, policy_summary
from .six_axis_force_drive import SixAxisForceDrive
from .process_recording import AsyncRecords, AsyncSerialTrace


class JointReferenceState(CommandState):
    """Retain only the newest whole IK reference, never queue force commands."""
    latest=None
    def ingest(self,rows,now_ns,anchor):
        forwarded=[];newest=None
        for row in rows:
            if row.get('mode')!='joint_position' or row.get('command') is None:
                forwarded.append(row);continue
            # Reuse J14 epoch/permission/lease checks without interpreting q as force.
            forwarded.append(dict(row,mode='static',command=None))
            cmd=row['command'];seq=cmd.get('seq');stamp=cmd.get('publish_ns')
            if type(seq) is not int or type(stamp) is not int or not 0<stamp<=now_ns:
                raise ValueError('IK reference sequence/timestamp invalid')
            q=cmd.get('target_m')
            if not isinstance(q,list) or len(q)!=6 or not np.isfinite(q).all():
                raise ValueError('complete six-axis finite IK reference required')
            if seq>self.seq and (newest is None or seq>newest['seq']):newest=cmd
        super().ingest(forwarded,now_ns,anchor)
        if newest is not None and not self.stop_reason:
            self.latest=dict(newest,receive_ns=now_ns);self.seq=newest['seq'];self.command_ns=newest['publish_ns'];self.receive_ns=now_ns


class JointPositionServoCore(ServoCore):
    def __init__(self,config,bus=None):
        limits=force_limits()
        config=dict(config,min_force_n=list(limits.min_force_n),max_force_n=list(limits.max_force_n))
        super().__init__(config,bus)
        self.bus.timeout_s=.05  # same Force maintenance RPC timeout as the existing entry
        # This adapter records the existing J14 timing/health fields below;
        # do not emit an empty legacy recorder as a zero-stall measurement.
        self.rec.enabled=False;self.rec._dumped=True
        self.records=AsyncRecords(self.out);self.trace=None
        if self.serial is not None:
            self.trace=AsyncSerialTrace(self.serial,self.out,self.records)
            self.serial=self.bus.serial=self.trace
        self.geometry=load_geometry_config();self.pid=None;self.pending=None;self.last_complete=None
        self.started_ns=None;self.monitor_index=0;self.monitor_ns=0;self.snapshot=None
        self.normal_cycles=0;self.input_cycle_seq=-1;self.modes=[0]*6
        self.recovery=None
        self.accepted_reference=None
        self.reference_ready_ns=None

    def initialize(self):
        ready=super().initialize()  # maintenance_only: actual Mode 0, no motion write
        self._read_positions()
        self.initial=list(self.positions)
        self.device_limits=[(Motor(self.bus,i).read_register(0x24).values[0],
                             Motor(self.bus,i).read_register(0x23).values[0]) for i in range(1,7)]
        # Read existing hardware settings, never rewrite or infer a force clamp.
        self.device_protection=[dict(motor_id=i,overcurrent_ma=Motor(self.bus,i).read_register(0x20).values[0],
            forward_output_raw_u16=Motor(self.bus,i).read_register(0x21).values[0],
            reverse_output_raw_u16=Motor(self.bus,i).read_register(0x22).values[0]) for i in range(1,7)]
        g=self.geometry
        self.lower=np.array([max(g.normal_motion_min_steps,g.absolute_raw_min_steps,x[0]) for x in self.device_limits])
        self.upper=np.array([min(g.normal_motion_max_steps,g.absolute_raw_max_steps,g.software_command_max_steps,x[1]) for x in self.device_limits])
        self._check_positions(self.positions)
        self.hold=self.prepare_force_handover()['holding_forces_g']
        self.command=JointReferenceState(self.config['epoch'],self.positions)
        ready.update(positions=self.initial,device_limits=self.device_limits,hold_forces_g=self.hold,
            device_protection_registers=self.device_protection,output_policy=policy_summary())
        return ready

    def _check_positions(self,positions):
        q=np.asarray(positions)
        if (q<self.lower).any() or (q>self.upper).any():
            raise RuntimeError('ACTUAL_POSITION_MECHANICAL_BOUND: '+str(list(positions)))

    def _ingest(self,statuses,stamps,start):
        if set(statuses)!=set(range(1,7)) or set(stamps)!=set(range(1,7)):
            raise RuntimeError('incomplete six-axis status response')
        feedback=self.feedback.ingest_cycle_feedback(statuses,servo_cycle_id=self.normal_cycles,
            servo_cycle_timestamp_ns=start,per_axis_rx_done_ns=stamps)
        self.update(feedback)
        if self.feedback.stopped or self.writer.stopped or not all(self.force_valid):
            raise RuntimeError('communication/status feedback stopped or invalid')
        if any(self.faults):raise RuntimeError('motor fault: '+str(self.faults))
        if max(map(abs,self.currents))>self.config['max_current_ma']:
            raise RuntimeError(f"{self.config['max_current_ma']} mA current protection exceeded")
        if any(not 0<=time.monotonic_ns()-t<=200_000_000 for t in self.rx_ns):
            raise RuntimeError('missing/stale actual position feedback')
        self._check_positions(self.positions)

    def _read_positions(self):
        start=time.monotonic_ns();statuses={};stamps={}
        with self.writer.uart_lock:
            for i in range(1,7):
                statuses[i]=Motor(self.bus,i).read_status();stamps[i]=time.monotonic_ns()
        # During initial creation the mechanical limits are not available yet.
        if not hasattr(self,'lower'):
            feedback=self.feedback.ingest_cycle_feedback(statuses,servo_cycle_id=0,
                servo_cycle_timestamp_ns=start,per_axis_rx_done_ns=stamps)
            self.update(feedback)
        else:self._ingest(statuses,stamps,start)

    def _inventory(self):
        with self.writer.uart_lock:
            return [dict(mode=Motor(self.bus,i).read_register(0x25).values[0],
                target_g=self._signed(Motor(self.bus,i).read_register(0x27).values[0])) for i in range(1,7)]

    @staticmethod
    def _signed(value):return value-65536 if value>=32768 else value

    def _enter(self,cmd):
        self.records.submit('child_lifecycle.jsonl',dict(event='mode3_handover_begin',
            ns=time.monotonic_ns(),**self._reference_state(cmd)))
        def verify(_):
            self._read_positions()
            if max(abs(a-b) for a,b in zip(self.positions,self.initial))>6:
                raise RuntimeError('Mode 3 handover displacement exceeds 6 steps')
        self.enter_force_mode_with_handover(self.hold,static_baseline_g=self.hold,axis_verifier=verify)
        inventory=self._inventory()
        if [r['mode'] for r in inventory]!=[3]*6 or [r['target_g'] for r in inventory]!=list(self.hold):
            raise RuntimeError('Mode 3 / holding force readback mismatch')
        self.modes=[3]*6;self._read_positions()
        self.pid=JointPositionForcePID(JointForcePIDConfig(**cmd['pid_config']),clock=time.monotonic_ns,
                                       force_slew_compensation=False)
        now=time.monotonic_ns()
        self.pid.reset(raw_steps_to_slider_m(self.positions,self.geometry),sample_ns=max(self.rx_ns),
            now_ns=now,position_stamps_ns=self.rx_ns)
        self.drive=SixAxisForceDrive(None,force_limits(),clock=time.monotonic_ns)
        self.drive.anchor_hold_forces_g(self.hold,now_ns=now)
        self._read_positions();self.started_ns=time.monotonic_ns();self.monitor_ns=self.started_ns
        self.records.submit('child_lifecycle.jsonl',dict(event='mode3_confirmed_waiting_reference',
            ns=self.started_ns,**self._reference_state(cmd)))

    def _reference_state(self,cmd):
        cmd=cmd or {}
        return dict(command_seq=cmd.get('seq'),command_publish_ns=cmd.get('publish_ns'),
            command_receive_ns=cmd.get('receive_ns'),parent_lease_ns=self.command.lease_ns,
            mode3_start_ns=self.started_ns,reference_ready_ns=self.reference_ready_ns)

    def _monitor_one(self):
        now=time.monotonic_ns()
        if now-self.monitor_ns<1_000_000_000/12:return None
        mid=self.monitor_index//2+1;reg=0x25 if self.monitor_index%2==0 else 0x27
        expected=3 if reg==0x25 else self.last_command_force_g[mid-1]
        with self.writer.uart_lock:value=Motor(self.bus,mid).read_register(reg).values[0]
        if reg==0x27:value=self._signed(value)
        if value!=expected:raise RuntimeError('Mode 3 lost during position control' if reg==0x25 else '0x27 target force readback mismatch')
        self.monitor_ns+=int(1_000_000_000/12);self.monitor_index=(self.monitor_index+1)%12
        return dict(motor_id=mid,register=reg,value=value,expected=expected,ns=time.monotonic_ns())

    def _packet(self,**fields):
        now=time.monotonic_ns()
        ref=self.accepted_reference or {}
        return dict(type='feedback',epoch=self.config['epoch'],cycle_seq=self.normal_cycles-1,
            feedback_ready_ns=now,cycle_start_ns=now,cycle_end_ns=now,command_seq=ref.get('seq',-1),
            command_publish_ns=ref.get('publish_ns'),command_receive_ns=ref.get('receive_ns'),
            positions=list(self.positions),currents=list(self.currents),faults=list(self.faults),rx_ns=list(self.rx_ns),
            targets=list(self.positions),tx_start_ns=[0]*6,tx_end_ns=[0]*6,control_mode=3 if self.pid else 0,
            mode3_start_ns=self.started_ns,reference_ready_ns=self.reference_ready_ns,
            **self.force_feedback_fields(),**fields)

    def cycle(self,expected_ns,hold_for_ipc_backlog=False):
        now=time.monotonic_ns()
        if not hold_for_ipc_backlog:self.accepted_reference=self.command.latest
        cmd=self.accepted_reference
        try:
            if self.pid is not None and self.reference_ready_ns is None:
                # Mode 3 entry performs blocking UART confirmation. References
                # queued during it are not post-entry control input. Drain them
                # through the existing J14 loop, letting Parent refill its latest
                # slot; never re-date or dispatch an old vector. The same 200 ms
                # timeout bounds this wait, starting at confirmed entry completion.
                if now-self.started_ns>self.command.timeout_ns:
                    raise RuntimeError('IK_target_timeout: no fresh post-handover reference; terminal handover required')
                if cmd is None or cmd['publish_ns']<self.started_ns or hold_for_ipc_backlog:
                    self._read_positions()  # retain actual feedback/protections at holding bias
                    return self._packet()
            if cmd is None:return self._packet()
            if now-cmd['publish_ns']>self.command.timeout_ns:
                raise RuntimeError('IK_target_timeout: terminal handover required')
            imu=cmd['attitude']['imu']
            if any(not 0<=now-imu.get(name+'_timing',{}).get('latest_rx_ns',imu['sample_ns'])<=
                   self.config.get('imu_age_limits_s',{}).get(name,.1)*1e9
                   for name in ('base','platform')):
                raise RuntimeError('IMU orientation stale in Child')
            if self.pid is None:
                self._enter(cmd)
                return self._packet()  # admit a fresh queued IMU/IK command after the handover
            if self.reference_ready_ns is None:
                self.reference_ready_ns=now
                self.records.submit('child_lifecycle.jsonl',dict(event='ik_reference_ready',
                    ns=now,**self._reference_state(cmd)))
            self.pending=dict(ns=now,phase=cmd['phase'],phase_elapsed_s=cmd['phase_elapsed_s'],stage='pid_update',
                target_pose_m_rad=cmd['target_pose_m_rad'],target_positions_steps=cmd['target_steps'],
                actual_positions_steps=list(self.positions),
                position_error_steps=(np.array(cmd['target_steps'])-self.positions).tolist(),pid=None,target_force_g=None)
            self._check_positions(self.positions)
            q=raw_steps_to_slider_m(self.positions,self.geometry);stamps=list(self.rx_ns)
            delta,diag=self.pid.update(cmd['target_m'],q,sample_ns=max(stamps),now_ns=now,position_stamps_ns=stamps)
            self.pending.update(pid=diag,stage='force_write')
            command=self.drive.command_forces_n(np.array(g_to_n(self.hold))+delta,dry_run=True,now_ns=now,
                braking_axes=self.pid.braking_axes)
            forces=[f['target_force_g'] for f in command['frames']];self.pending['target_force_g']=forces
            self.check_hardware_write_permission()
            audit=self.writer.send_force_cycle(forces,cycle_id=self.normal_cycles,target_timestamp_ns=now)
            self.normal_cycles+=1
            self.last_command_force_g=list(forces)
            final_integral_blocked=self.pid.track_applied(np.array(g_to_n(forces))-np.array(g_to_n(self.hold)))
            force_limited=(abs(np.array(command['requested_forces_n'])-command['limited_forces_n'])>1e-10).tolist()
            self.pending['stage']='feedback_ingest'
            self._ingest(audit.feedbacks,audit.feedback_rx_done_ns,audit.cycle_start_ns)
            self.pending['stage']='mode_and_force_readback'
            monitor=self._monitor_one()
            row=dict(ns=time.monotonic_ns(),phase=cmd['phase'],phase_elapsed_s=cmd['phase_elapsed_s'],
                source='REAL_HARDWARE' if self.config['hardware_mode']=='active' else 'MOCK',
                target_timestamp_ns=now,reference_timestamp_ns=cmd['publish_ns'],command_seq=cmd['seq'],
                expected_start_ns=expected_ns,wake_lateness_ns=max(0,now-expected_ns),
                tx_start_ns=[r.tx_start_ns for r in audit.tx_records],tx_end_ns=[r.tx_end_ns for r in audit.tx_records],
                batch_complete_ns=audit.batch_complete_ns,tx_skew_ns=audit.tx_skew_ns,
                pid_input_cycle_seq=self.input_cycle_seq,uart_cycle_sequence=self.normal_cycles-1,
                reference_observed_uart_cycle_seq=cmd.get('observed_uart_cycle_sequence'),
                target_pose_m_rad=cmd['target_pose_m_rad'],target_positions_steps=cmd['target_steps'],
                actual_positions_steps=list(self.positions),position_error_steps=(np.array(cmd['target_steps'])-self.positions).tolist(),
                pid=diag,pid_input_rx_ns=stamps,pid_output_g=(delta/NOMINAL_N_PER_G).tolist(),
                target_force_g=forces,target_force_27_g_readback=None,register_monitor=monitor,
                absolute_force_limited_axes=force_limited,final_integral_blocked_axes=final_integral_blocked,
                force_2c_g=list(self.forces_g_raw),currents_ma=list(self.currents),fault_codes=list(self.faults),
                force_adc_raw=list(self.force_adc_raw),force_valid=list(self.force_valid),
                modes_last_readback=list(self.modes),feedback_rx_ns=list(self.rx_ns),
                attitude=cmd['attitude'],cycle_ms=(time.monotonic_ns()-now)/1e6)
            self.last_complete=row;self.input_cycle_seq=self.normal_cycles-1;self.cycle_seq+=1
            self.health.update(cycles=self.normal_cycles,motion_writes=self.normal_cycles*6)
            self.health['max_wake_lateness_ns']=max(self.health['max_wake_lateness_ns'],max(0,now-expected_ns))
            self.health['wake_watchlist_count']+=int(now-expected_ns>30_000_000)
            self.health['max_rx_drain_ns']=max(self.health['max_rx_drain_ns'],audit.motion_drain_wall_ns)
            for tx in audit.tx_records:
                if tx.serial_write_metric is not None:
                    self.health['max_serial_write_ns']=max(self.health['max_serial_write_ns'],tx.serial_write_metric.wall_duration_ns)
            self.rows.append(dict(cycle_seq=self.normal_cycles-1,expected_ns=expected_ns,start_ns=now,end_ns=row['ns'],
                wake_lateness_ns=max(0,now-expected_ns),command_seq=cmd['seq'],command_publish_ns=cmd['publish_ns'],
                command_receive_ns=cmd.get('receive_ns'),fault=None))
            return self._packet(motion=row)
        except Exception as exc:
            self.health['fault_cycles']+=1
            self.health['tx_timeouts']+=int(type(exc).__name__=='J7AdapterWriteTimeout')
            self.snapshot=dict(fault_timestamp_ns=time.monotonic_ns(),error=f'{type(exc).__name__}: {exc}',
                reference=self._reference_state(cmd),
                pending_cycle=self.pending,last_complete_frame=self.last_complete,
                attitude=cmd.get('attitude') if cmd else None,
                target_request=cmd.get('target_request') if cmd else None,
                latest_feedback=dict(actual_positions_steps=list(self.positions),feedback_rx_ns=list(self.rx_ns),
                    currents_ma=list(self.currents),force_2c_g=list(self.forces_g_raw),fault_codes=list(self.faults)))
            self.stop(f'{type(exc).__name__}: {exc}')
            return dict(type='fault',epoch=self.config['epoch'],communication_fault=self.fault)

    def stop(self,reason):
        if self.fault:return
        self.fault=reason;self.motion_stop_ns=time.monotonic_ns()
        outcome='operator_stopped' if reason=='operator_stopped' else 'motion_complete' if reason=='normal_exit' else 'fault'
        if outcome=='fault' and self.snapshot is None:
            state=getattr(self,'command',None)
            cmd=state.latest if hasattr(state,'latest') else None
            self.snapshot=dict(fault_timestamp_ns=self.motion_stop_ns,error=reason,pending_cycle=self.pending,
                reference=self._reference_state(cmd),
                last_complete_frame=self.last_complete,attitude=cmd.get('attitude') if cmd else None,
                target_request=cmd.get('target_request') if cmd else None,
                latest_feedback=dict(actual_positions_steps=list(self.positions),feedback_rx_ns=list(self.rx_ns),
                    currents_ma=list(self.currents),force_2c_g=list(self.forces_g_raw),fault_codes=list(self.faults)))
        if self.pid is not None and self.mode_3_active:
            self.recovery=recover_holding_bias(core=self,drive=self.drive,hold=self.hold,outcome=outcome,error=reason,
                first_cycle_id=self.normal_cycles,read_positions=self._read_positions,inventory=self._inventory,
                ingest=self._ingest,check_positions=self._check_positions,record=self.records.submit)
        self.finish_force_control(reason,fault=outcome=='fault')

    def close(self,reason='shutdown'):
        if self.closed:return
        super().close(reason)
        record_health=self.records.close()
        terminal=self.force_shutdown_report
        report=dict(control_cycles=self.normal_cycles,error=None if self.fault=='normal_exit' else self.fault,
            outcome='motion_complete' if self.fault=='normal_exit' else 'operator_stopped' if self.fault=='operator_stopped' else 'fault',
            all_mode3_confirmed=self.pid is not None,all_mode0_exit_confirmed=bool(terminal and terminal.get('position_hold_confirmed')),
            serial_closed=self.closed,hold_forces_g=self.hold if hasattr(self,'hold') else None,
            pid=asdict(self.pid.config) if self.pid else None,force_recovery=self.recovery,
            terminal_recovery=terminal,recording=record_health,
            hardware_write_count=self.trace.write_count if self.trace else 0,source='REAL_HARDWARE' if self.trace else 'MOCK')
        report.update(output_policy=policy_summary(),device_protection_registers=getattr(self,'device_protection',None))
        if self.snapshot:
            self.snapshot.update(all_mode0_exit_confirmed=report['all_mode0_exit_confirmed'],serial_closed=self.closed)
            (self.out/'fault_snapshot.json').write_text(json.dumps(self.snapshot,indent=2)+'\n')
        (self.out/'motion_child_report.json').write_text(json.dumps(report,indent=2)+'\n')
