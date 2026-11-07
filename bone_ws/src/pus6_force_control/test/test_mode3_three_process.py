"""J14 process adaptation, direct PI-D and peer failure; Mock motors only."""
from dataclasses import asdict, replace
import json
from pathlib import Path
import threading
import time

import numpy as np
import pytest

from pus6_force_control.mode3_process_child import JointReferenceState, JointPositionServoCore
from pus6_force_control.mode3_process_runtime import run
from pus6_force_control.mode3_dual_imu import DualImuPlan, DualImuMotionConfig, SyntheticImuSource
from pus6_force_control.dual_imu_attitude import load_attitude_parameters
from pus6_force_control.z_position_motion import pid_config, slider_to_steps
from pus6_force_control.joint_position_force_pid import JointPositionForcePID
from pus6_force_control.mode3_process_policy import PID_OVERRIDES
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j14_process_uart import ProcessClient


def configuration(out):
    return dict(epoch=time.monotonic_ns(),hardware_mode='mock',output_dir=str(out),
        execute=True,dry_run=False,hardware_write_enabled=True,is_hardware_qualified=True,
        control_mode='force',maintenance_only=True,force_fault_recovery='position_reanchor',
        max_force_handover_displacement_steps=6,max_current_ma=1500)


def packet(plan,seq=1):
    pose,q=plan.target('initial_hold',0)
    return dict(seq=seq,publish_ns=time.monotonic_ns(),target_m=q.tolist(),
        target_steps=slider_to_steps(q,plan.geometry).tolist(),target_pose_m_rad=pose.tolist(),
        phase='initial_hold',phase_elapsed_s=0,attitude=plan.control_diagnostics,
        target_request=plan.target_request,pid_config=asdict(replace(pid_config(plan),**PID_OVERRIDES)))


def test_latest_complete_reference_epoch_sequence_and_original_timeout():
    state=JointReferenceState(12,[1000]*6)
    def p(seq,stamp=1_000_000_000):
        return dict(type='state',epoch=12,mode='joint_position',allowed=True,lease_ns=stamp,
            command=dict(seq=seq,publish_ns=stamp,target_m=[.05]*6))
    state.ingest([p(5),p(2),p(7),p(6)],1_010_000_000,[1000]*6)
    assert state.seq==7 and state.latest['seq']==7
    state.ingest([p(3)],1_020_000_000,[1000]*6)
    assert state.seq==7
    state.ingest([dict(p(8),command=None,mode='static')],1_200_000_001,[1000]*6)
    assert state.command_ns==1_000_000_000  # lease cannot renew stale target time
    assert state.timeout_ns==200_000_000
    with pytest.raises(ValueError,match='epoch'):
        state.ingest([dict(p(9),epoch=13)],1_210_000_000,[1000]*6)


def test_slow_recording_is_bounded_and_does_not_block_the_producer(tmp_path):
    from pus6_force_control.process_recording import AsyncRecords
    entered=threading.Event();release=threading.Event()
    def slow(name,row):entered.set();release.wait(2);return row
    sink=AsyncRecords(tmp_path,capacity=2,transform=slow)
    sink.submit('records.jsonl',dict(index=0));assert entered.wait(1)
    for i in range(1,5):sink.submit('records.jsonl',dict(index=i))
    assert sink.queue.qsize()==2 and sink.dropped==2
    release.set();health=sink.close()
    assert health==dict(dropped=2,error=None,closed=True)
    assert len((tmp_path/'records.jsonl').read_text().splitlines())==3


@pytest.mark.parametrize('fault',['mechanical','target_age','imu'])
def test_child_original_guards_capture_frame_and_restore_mode0(tmp_path,fault,monkeypatch):
    import serial
    monkeypatch.setattr(serial,'Serial',lambda *a,**kw:pytest.fail('Mock opened serial'))
    core=JointPositionServoCore(configuration(tmp_path),MockServoBus(initial_positions=[1000]*6,initial_modes=[0]*6))
    try:
        ready=core.initialize()
        assert ready['positions']==[1000]*6
        plan=DualImuPlan([1000]*6,source=SyntheticImuSource(),parameters=load_attitude_parameters())
        core.command.latest=packet(plan);core.cycle(time.monotonic_ns())
        core.command.latest=packet(plan,2);core.cycle(time.monotonic_ns())
        cmd=packet(plan,3)
        if fault=='mechanical':core.positions[0]=1801
        elif fault=='target_age':cmd['publish_ns']-=200_000_001
        else:cmd['attitude']['imu']['sample_ns']-=100_000_001
        core.command.latest=cmd
        result=core.cycle(time.monotonic_ns())
        assert result['type']=='fault'
    finally:core.close()
    r=json.loads((tmp_path/'motion_child_report.json').read_text())
    assert r['all_mode0_exit_confirmed'] and r['hardware_write_count']==0 and r['serial_closed']
    assert (tmp_path/'fault_snapshot.json').exists()
    assert {'mechanical':'ACTUAL_POSITION_MECHANICAL_BOUND','target_age':'IK_target_timeout','imu':'IMU orientation stale'}[fault] in r['error']


def test_child_pi_d_matches_existing_algorithm_without_reference_or_force_slew(tmp_path):
    core=JointPositionServoCore(configuration(tmp_path),MockServoBus(initial_positions=[1000]*6,initial_modes=[0]*6))
    try:
        core.initialize();plan=DualImuPlan([1000]*6,source=SyntheticImuSource(),parameters=load_attitude_parameters())
        core.command.latest=packet(plan);core.cycle(time.monotonic_ns())
        core.command.latest=packet(plan,2);core.cycle(time.monotonic_ns())
        native_config=replace(pid_config(plan),**PID_OVERRIDES)
        reference=JointPositionForcePID(native_config,clock=time.monotonic_ns,force_slew_compensation=False)
        reference.reset(core.pid.previous_q,sample_ns=core.pid.previous_sample_ns,now_ns=time.monotonic_ns(),
            position_stamps_ns=core.pid.position_history_previous_ns)
        cmd=packet(plan,3);cmd['target_m'][0]+=.001;cmd['target_steps'][0]+=40
        core.command.latest=cmd;core.command.seq=3
        held=core.cycle(time.monotonic_ns(),hold_for_ipc_backlog=True)['motion']
        assert held['command_seq']==2 and held['target_force_g']==[0]*6
        # Match the same retained measured-state history after the held cycle.
        reference=JointPositionForcePID(native_config,clock=time.monotonic_ns,force_slew_compensation=False)
        import copy
        for name,value in vars(core.pid).items():
            if name not in ('clock','config','reference'):setattr(reference,name,copy.deepcopy(value))
        before_stamps=list(core.rx_ns);q=core.pid.previous_q.copy()
        row=core.cycle(time.monotonic_ns())['motion']
        expected,diag=reference.update(cmd['target_m'],q,sample_ns=max(before_stamps),
            now_ns=row['target_timestamp_ns'],position_stamps_ns=before_stamps)
        assert row['pid']['joint_delta_force_n']==pytest.approx(expected)
        assert row['pid']['joint_reference_m']==pytest.approx(cmd['target_m'],abs=0)
        assert row['target_force_g'][0]>408  # no former 4 N cap on the first reference step
        assert row['pid']['joint_d_n']==pytest.approx(diag['joint_d_n'])
        assert core.drive.limits.max_rate_g_s is None
        assert core.pid.config==native_config
    finally:core.stop('normal_exit');core.close()


def test_post_handover_wait_is_bounded_and_lease_cannot_refresh_ik(tmp_path):
    core=JointPositionServoCore(configuration(tmp_path),MockServoBus(initial_positions=[1000]*6,initial_modes=[0]*6))
    try:
        core.initialize();plan=DualImuPlan([1000]*6,source=SyntheticImuSource(),parameters=load_attitude_parameters())
        cmd=packet(plan);core.command.latest=cmd
        core.cycle(time.monotonic_ns())
        assert core.pid is not None and core.reference_ready_ns is None
        assert core.cycle(time.monotonic_ns())['type']=='feedback'
        assert core.normal_cycles==0  # no queued entry reference becomes a force cycle
        core.started_ns-=200_000_001
        # A live Parent heartbeat is not evidence of a fresh complete IK vector.
        now=time.monotonic_ns()
        core.command.ingest([dict(type='state',epoch=core.config['epoch'],mode='static',
            allowed=True,lease_ns=now,command=None)],now,core.positions)
        assert core.cycle(now)['type']=='fault'
        assert 'no fresh post-handover reference' in core.fault
        assert core.normal_cycles==0 and core.command.latest['publish_ns']==cmd['publish_ns']
    finally:core.close()
    report=json.loads((tmp_path/'motion_child_report.json').read_text())
    assert report['all_mode0_exit_confirmed'] and report['hardware_write_count']==0


@pytest.mark.parametrize('empty_file',[False,True])
def test_analysis_without_tracking_preserves_startup_fault(tmp_path,empty_file):
    from pus6_force_control.dual_imu_analysis import analyze_run
    error='RuntimeError: IK_target_timeout: terminal handover required'
    report=dict(source='MOCK',outcome='fault',error=error,hardware_write_count=0,
        all_mode0_exit_confirmed=True,serial_closed=True,fault_timestamp_ns=123)
    (tmp_path/'dual_imu_report.json').write_text(json.dumps(report))
    (tmp_path/'fault_snapshot.json').write_text(json.dumps(dict(pending_cycle=None,latest_feedback={},target_request={})))
    if empty_file:(tmp_path/'tracking.jsonl').touch()
    result=analyze_run(tmp_path,tmp_path/'analysis')
    assert result['error']==error and result['fault_timestamp_ns']==123
    assert result['completed_control_samples']==0 and result['completed_active_samples']==0
    assert result['analysis_status']=='NO_ACTIVE_SAMPLES' and result['tracking_available']==empty_file
    assert result['stability_verdict']=='NOT_ESTABLISHED' and not result['full_unscaled_target_completed']
    assert result['analysis_hardware_write_count']==0


def test_parent_command_channel_eof_stops_existing_child_locally(tmp_path):
    settings=DualImuMotionConfig()
    config=dict(configuration(tmp_path),production=True,feedback_bridge=True,joint_position_pid=True,
        enable_coordinated_handshake=True,telemetry_mode='production',telemetry_capacity_cycles=4096,
        max_servo_duration_s=0.,motion_settings=asdict(settings))
    client=ProcessClient(config)
    try:
        plan=DualImuPlan(client.ready['positions'],source=SyntheticImuSource(),parameters=load_attitude_parameters())
        deadline=time.monotonic()+3;started=False
        while time.monotonic()<deadline and not started:
            client.update_joint_reference(packet(plan));client.send_state()
            started=any(r.get('type')=='motion_status' and r.get('mode3_start_ns') for r in client.receive())
            time.sleep(.005)
        assert started
        import socket
        client.sock.shutdown(socket.SHUT_WR)  # Parent disappearance; no recovery writer in this process
        client.proc.wait(timeout=3)
    finally:client.close()
    report=json.loads((tmp_path/'motion_child_report.json').read_text())
    assert report['all_mode0_exit_confirmed'] and report['hardware_write_count']==0
    assert report['error']=='parent_ipc_closed'


@pytest.mark.parametrize('failure',[None,'slow_handover','bridge','child'])
def test_real_three_process_mock_execution_and_peer_failure(tmp_path,monkeypatch,failure):
    import pus6_force_control.mode3_process_runtime as runtime
    if failure=='slow_handover':
        # Reproduce the real >200 ms Mode 3 entry while Parent fills the same
        # bounded IPC socket. Inject delay in the test Child, never production.
        import pus6_motor_driver.j14_process_uart as uart
        original_popen=uart.subprocess.Popen
        wrapper=tmp_path/'slow_mock_child.py'
        wrapper.write_text("import runpy,sys,time\n"
            "entry=sys.argv.pop(1)\nsys.argv[0]=entry\n"
            "bootstrap=runpy.run_path(entry,run_name='mock_bootstrap')\n"
            "from pus6_force_control.mode3_process_child import JointPositionServoCore\n"
            "original=JointPositionServoCore._enter\n"
            "def slow(self,cmd):\n    time.sleep(.3)\n    return original(self,cmd)\n"
            "JointPositionServoCore._enter=slow\n"
            "bootstrap['main']()\n")
        def popen(argv,**kwargs):
            if argv[1].endswith('j14_uart_child_main.py'):
                argv=[argv[0],str(wrapper),*argv[1:]]
            return original_popen(argv,**kwargs)
        monkeypatch.setattr(uart.subprocess,'Popen',popen)
    clients=[];worker=[]
    def create(config):
        c=ProcessClient(config);clients.append(c)
        assert c.ready['pid']!=__import__('os').getpid()
        assert c.ready['modules_without_ros']
        if failure in ('bridge','child'):
            def kill_peer():
                deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    f=tmp_path/'tracking.jsonl'
                    if f.exists() and f.stat().st_size>0:break
                    time.sleep(.01)
                (c.bridge_proc if failure=='bridge' else c.proc).kill()
            t=threading.Thread(target=kill_peer);t.start();worker.append(t)
        return c
    monkeypatch.setattr(runtime,'ProcessClient',create)
    settings=DualImuMotionConfig(hold_seconds=.03,settle_seconds=.03,settle_timeout_s=1.)
    config=configuration(tmp_path)
    if failure in (None,'slow_handover'):config['mock_initial_positions']=[995,1007,1002,998,1003,1001]
    result=run(config,settings,source=SyntheticImuSource(duration_s=.15,amplitude_deg=.005),
        parameters=load_attitude_parameters(),axes=('roll','pitch','yaw'),duration=.15)
    for t in worker:t.join()
    assert result['hardware_write_count']==0
    assert clients[0].proc.poll() is not None and clients[0].bridge_proc.poll() is not None
    if failure=='child':
        assert result['outcome']=='exit_unconfirmed' and not result['all_mode0_exit_confirmed']
        return
    assert result['all_mode0_exit_confirmed'] and result['serial_closed']
    if failure=='bridge':
        assert result['outcome']=='fault'
        assert 'feedback_ipc_closed' in result['error'] or 'Feedback Bridge' in result['error']
    else:
        assert result['outcome']=='motion_complete'
        assert all(result['pid'][k] is None for k in PID_OVERRIDES)
        assert result['output_policy']['absolute_target_force_limit_n']==50
        assert np.rint(result['plan']['initial_steps']).astype(int).tolist()==config['mock_initial_positions']
        assert [s['phase'] for s in result['stages']]==['initial_hold','stabilize','final_hold']
        rows=[json.loads(x) for x in (tmp_path/'tracking.jsonl').read_text().splitlines()]
        assert len(rows)==result['control_cycles']
        assert [r['uart_cycle_sequence'] for r in rows]==list(range(len(rows)))
        assert [r['pid_input_cycle_seq'] for r in rows]==list(range(-1,len(rows)-1))
        assert all(r['command_seq']>=0 and r['reference_timestamp_ns']<=r['target_timestamp_ns'] for r in rows)
        assert result['bridge_recording']==dict(dropped=0,error=None,closed=True)
        events=[json.loads(x) for x in (tmp_path/'child_lifecycle.jsonl').read_text().splitlines()]
        entered=next(r for r in events if r['event']=='mode3_confirmed_waiting_reference')
        accepted=next(r for r in events if r['event']=='ik_reference_ready')
        assert rows[0]['reference_timestamp_ns']>=entered['ns']
        assert rows[0]['command_seq']==accepted['command_seq']>entered['command_seq']
        assert 0<=accepted['ns']-entered['ns']<=200_000_000
        assert result['stages'][0]['elapsed_s']>=1.
