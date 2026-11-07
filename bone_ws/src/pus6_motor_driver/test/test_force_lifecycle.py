"""Terminal handling on FakeBus only; no serial constructors or real processes."""
import time
import pytest
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j14_process_uart import ServoCore, CommandState
from pus6_motor_driver.protocol import parse_frame


def owner(tmp_path,bus=None):
    bus=bus or MockServoBus(initial_positions=[1000]*6)
    bus.modes={i:0 for i in range(1,7)}
    bus.targets=dict(bus.positions);bus.actual_forces={i:-100*i for i in range(1,7)}
    c=ServoCore(dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path),control_mode='force',
        maintenance_only=True,mode3_only=True,dry_run=False),bus=bus)
    c.initialize();hold=c.prepare_force_handover()['holding_forces_g']
    c.enter_force_mode_with_handover(hold,static_baseline_g=hold)
    return c,bus


@pytest.mark.parametrize('reason',['normal_exit','service_pause','parent_lease_expired',
    'parent_ipc_closed','feedback_invalid','signal_stop','child_exception'])
def test_every_terminal_path_anchors_current_and_is_idempotent(tmp_path,reason):
    c,b=owner(tmp_path)
    b.positions={i:1000+i for i in range(1,7)}
    c.stop(reason);n=len(b.write_audit);c.stop(reason);c.close()
    assert len(b.write_audit)==n
    assert [b.modes[i] for i in range(1,7)]==[0]*6
    assert b.targets==b.positions
    report=c.force_shutdown_report
    assert report['position_hold_confirmed'] and not report['independent_stop_required']
    assert all(a['anchor_steps']==1000+a['motor_id'] for a in report['axes'])
    assert not any(w['register_address'] in (9,0x29) for w in b.write_audit)


def test_latched_writer_still_exits_and_readonly_attach_does_not_write(tmp_path):
    c,b=owner(tmp_path/'owned');c.writer.latch_fault_without_pause('sensor fault')
    c.close();assert c.force_shutdown_report['position_hold_confirmed']
    b=MockServoBus();b.modes={i:3 for i in range(1,7)}
    c=ServoCore(dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path/'readonly'),
        control_mode='force',maintenance_only=True,mode3_only=True,dry_run=False),bus=b)
    c.initialize();c.close();assert b.write_audit==[]


@pytest.mark.parametrize('position',[65535,2004,-1])
def test_bad_anchor_never_written_or_assumed_home(tmp_path,position):
    c,b=owner(tmp_path);b.positions[4]=position
    c.stop('feedback_invalid');c.close()
    report=c.force_shutdown_report
    assert not report['hardware_stop_confirmed'] and report['independent_stop_required']
    assert report['axes'][3]['final_mode']==3
    assert report['axes'][3]['anchor_steps'] is None
    assert not any(w['register_address']==0x25 and w['motor_id']==4 and w['payload'][0]==0 for w in b.write_audit)
    assert sum(w['register_address']==0x1A for w in b.write_audit)==6
    assert all(b.modes[i]==0 for i in (1,2,3,5,6))


def test_lost_exit_ack_is_not_retried_or_reported_confirmed(tmp_path):
    class Lost(MockServoBus):
        def request(self,raw,**kw):
            f=parse_frame(raw)
            response=super().request(raw,**kw)
            if f.command==0x32 and f.register_addr==0x25 and f.motor_id==3 and f.data[:2]==b'\0\0':
                raise TimeoutError('executed but lost ACK')
            return response
    c,b=owner(tmp_path,Lost(initial_positions=[1000]*6));c.stop('normal_exit');c.close()
    writes=[w for w in b.write_audit if w['register_address']==0x25 and w['motor_id']==3 and w['payload'][0]==0]
    assert len(writes)==1
    assert not c.force_shutdown_report['hardware_stop_confirmed']
    assert c.force_shutdown_report['axes'][2]['final_mode']==0 # read-only inventory; ACK still unconfirmed


def test_zero_current_and_pause_ack_cannot_prove_mode3_stop(tmp_path):
    c,b=owner(tmp_path);b.positions[4]=65535
    r=c.pause_explicit();c.close()
    assert r['pause_acknowledged'] and not r['position_hold_confirmed']
    assert r['independent_stop_required']


def test_stale_force_command_is_not_zero_or_old_hold_fallback():
    cmd=CommandState(1,[1000]*6,hold_anchor_on_timeout=True,fallback_force_n=[1.]*6)
    cmd.command_ns=1_000_000_000
    with pytest.raises(RuntimeError,match='force_target_timeout'):
        cmd.candidate_force([2.]*6,False,1_300_000_000)


def test_no_initial_force_publisher_times_out_through_owner(tmp_path):
    c,b=owner(tmp_path);c.force_entry_ns=time.monotonic_ns()-1_000_000_000
    packet=c.cycle(time.monotonic_ns());c.close()
    assert packet['type']=='fault'
    assert 'force_target_timeout' in c.fault
    assert c.force_shutdown_report['position_hold_confirmed']


def test_recorded_position_frames_preserve_signedness_and_physical_bounds():
    import csv
    from pathlib import Path
    from pus6_motor_driver.protocol import unpack_read_register_response
    repo=Path(__file__).resolve().parents[4]
    path=repo/'results/stage5_position_force_v2/20261104_175411_axes456_decoupled/register_reads.csv'
    rows=list(csv.DictReader(path.open()))
    pos={int(r['motor_id']):r for r in rows if int(r['register'])==0x2A}
    raw4=bytes.fromhex(pos[4]['rx_hex']);raw5=bytes.fromhex(pos[5]['rx_hex'])
    assert unpack_read_register_response(raw4,0x2A,1).values[0]==65535
    assert int.from_bytes(parse_frame(raw4).data,'little',signed=True)==-1
    assert unpack_read_register_response(raw5,0x2A,1).values[0]==186
    # Saved Mode3/force target fields are not position measurements.
    assert all(int(r['command'])==0x31 for r in pos.values())


@pytest.mark.parametrize('disconnect',['eof','lease'])
def test_existing_child_ipc_loss_runs_force_terminal_handler(tmp_path,disconnect):
    import json
    import socket
    import threading
    from pus6_motor_driver.j14_process_uart import child_run, receive_packets
    parent,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET);parent.setblocking(False);child.setblocking(False)
    bus=MockServoBus(initial_positions=[1000]*6);bus.modes={i:0 for i in range(1,7)}
    cfg=dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path),control_mode='force',
        dry_run=False,target_timeout_sec=5.,max_servo_duration_s=2.)
    thread=threading.Thread(target=child_run,args=(child,cfg,bus));thread.start()
    deadline=time.monotonic()+2.;rows=[]
    while time.monotonic()<deadline:
        batch,_=receive_packets(parent);rows+=batch
        if any(r['type']=='child_ready' for r in rows):break
        time.sleep(.005)
    assert any(r['type']=='child_ready' for r in rows)
    if disconnect=='eof':parent.close()
    thread.join(timeout=4.)
    assert not thread.is_alive()
    if disconnect=='lease':parent.close()
    final=json.loads((tmp_path/'child_final.json').read_text())
    assert final['fault']==('parent_ipc_closed' if disconnect=='eof' else 'parent_lease_expired')
    assert final['force_shutdown_report']['position_hold_confirmed']
    assert final['hardware_write_count']==0


def test_stage5_allowlist_allows_transport_internal_partial_entry_recovery(tmp_path):
    from pus6_force_control.stage5_experiment import Mode3OnlyTrace
    class Port:
        def write(self, raw):return len(raw)
        def close(self):pass
    trace=Mode3OnlyTrace(Port(),tmp_path)
    class Failure(MockServoBus):
        def request(self, raw, **kw):
            f=parse_frame(raw)
            if f.command==0x32:
                trace.write(raw)
                if f.motor_id==3 and f.register_addr==0x25 and f.data[:2]==b'\x03\0':
                    raise TimeoutError('entry failure')
            return super().request(raw,**kw)
    b=Failure(initial_positions=[1000]*6);b.modes={i:0 for i in range(1,7)}
    b.targets=dict(b.positions)
    c=ServoCore(dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path),control_mode='force',
        maintenance_only=True,mode3_only=True,dry_run=False),bus=b)
    c.serial=trace
    try:
        c.initialize()
        with pytest.raises(Exception):c.enter_force_mode_with_handover([-100]*6)
        assert [b.modes[i] for i in range(1,7)]==[0]*6
        assert not any(w['register_address']==0x1A for w in b.write_audit)
        assert c.writer.last_force_handover['recovery_complete']
    finally:c.close()


def test_complete_uart_loss_records_unknown_axes_and_never_retries_stop_writes(tmp_path):
    class Broken(MockServoBus):
        broken=False
        failed_requests=[]
        def request(self, raw, **kw):
            if self.broken:
                self.failed_requests.append((parse_frame(raw),kw))
                raise OSError('USB EIO')
            return super().request(raw,**kw)
    c,b=owner(tmp_path,Broken(initial_positions=[1000]*6));b.broken=True
    c.stop('communication_fault');count=len(b.failed_requests)
    c.stop('communication_fault');c.close()
    assert len(b.failed_requests)==count
    pause=[(f,kw) for f,kw in b.failed_requests if f.command==0x32]
    assert [f.motor_id for f,_ in pause]==list(range(1,7))
    assert all(f.register_addr==0x1A and kw['retries']==0 for f,kw in pause)
    r=c.force_shutdown_report
    assert not r['pause_acknowledged'] and r['independent_stop_required']
    assert all(a['final_mode'] is None and not a['anchor_confirmed'] for a in r['axes'])
