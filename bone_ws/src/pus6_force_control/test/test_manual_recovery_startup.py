"""Regression through Motor's actual StatusFeedback parser; never opens serial."""
from dataclasses import fields
import io
from types import SimpleNamespace

import pytest
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.protocol import StatusFeedback, parse_frame
from pus6_force_control.manual_recovery import read_manual_snapshot, manual_stop_status, run_manual

Q=[1955,350,122,232,934,2004]
HOLD=[-394,143,-300,-580,247,-480]


class StatusBus(MockServoBus):
    bad=None
    def __init__(self):
        super().__init__(initial_positions=Q)
        self.modes={i:0 for i in range(1,7)}
        self.targets={i:min(2000,max(0,q)) for i,q in enumerate(Q,1)}
        self.actual_forces=dict(enumerate(HOLD,1))
        self.target_forces={i:-1 for i in range(1,7)}
    def request(self, raw, **kw):
        frame=parse_frame(raw)
        if self.bad=='timeout' and frame.command==0x30 and frame.motor_id==2:
            raise TimeoutError('no status frame')
        reply=bytearray(super().request(raw,**kw))
        if frame.command==0x30:
            reply[13:15]=int(self.actual_forces[frame.motor_id]).to_bytes(2,'little',signed=True)
            if self.bad=='adc' and frame.motor_id==2:reply[15:17]=(4096).to_bytes(2,'little')
            reply[-1]=sum(reply[2:-1])&255
            if self.bad=='checksum' and frame.motor_id==2:reply[-1]^=1
        return bytes(reply)


def snapshot_core(bus):
    return SimpleNamespace(bus=bus,writer=SimpleNamespace(uart_lock=bus.uart_lock),
                           config={'max_current_ma':200},_force_control_managed=False)


@pytest.fixture(autouse=True)
def block_real_serial(monkeypatch):
    import serial
    def forbidden(*a,**kw):raise AssertionError('test must never open a real serial device')
    monkeypatch.setattr(serial,'Serial',forbidden)


def test_wire_status_has_no_scheduler_fields_and_startup_reads_all_six():
    assert {f.name for f in fields(StatusFeedback)}=={
        'target_position','current_position','current_mA','force_g','force_adc','temperature','error_code'}
    b=StatusBus();row=read_manual_snapshot(snapshot_core(b))
    assert row['completed'] and row['invalid_axis_ids']==[]
    assert [a['position_steps'] for a in row['axes']]==Q
    assert [a['force_g'] for a in row['axes']]==HOLD
    assert [a['force_2c_g'] for a in row['axes']]==HOLD
    assert all(a['mode']==0 and a['target_force_g']==-1 for a in row['axes'])
    assert b.write_audit==[]


@pytest.mark.parametrize('kind',['timeout','checksum','adc','fault','current','position'])
def test_bad_or_missing_feedback_raises_and_preserves_partial_data(kind):
    b=StatusBus();b.bad=kind
    if kind=='fault':b.faults[2]=1
    if kind=='current':b.currents[2]=201
    if kind=='position':b.positions[2]=2006
    row={}
    with pytest.raises(Exception):read_manual_snapshot(snapshot_core(b),row)
    assert not row['completed'] and row['invalid_axis_ids']==[2]
    assert row['error'] and row['axes'][0]['mode']==0
    assert b.write_audit==[]
    report=manual_stop_status(snapshot_core(b),None,row)
    assert not report['hardware_stop_confirmed'] and report['independent_stop_required']


def test_stop_report_separates_not_entered_unknown_failed_and_confirmed():
    b=StatusBus();c=snapshot_core(b);row=read_manual_snapshot(c)
    assert manual_stop_status(c,None,row)==dict(hardware_stop_confirmed=False,
        independent_stop_required=False,stop_status='NO_MODE3_ENTRY_MODE0_OBSERVED')
    assert manual_stop_status(c,None,None)['independent_stop_required']
    c._force_control_managed=True
    assert manual_stop_status(c,None,row)['independent_stop_required']
    terminal=dict(status='MODE0_ANCHORED_OBSERVED',hardware_stop_confirmed=True,independent_stop_required=False)
    assert manual_stop_status(c,terminal,None)['hardware_stop_confirmed']
    terminal.update(hardware_stop_confirmed=False) # Mock terminal observation never becomes hardware PASS
    result=manual_stop_status(c,terminal,None)
    assert not result['hardware_stop_confirmed'] and not result['independent_stop_required']
    terminal.update(status='STOP_UNCONFIRMED',independent_stop_required=True)
    assert manual_stop_status(c,terminal,None)['independent_stop_required']


@pytest.mark.parametrize('startup_failure',[False,True,'partial','barrier'])
def test_full_manual_entry_with_actual_core_fakebus_and_protocol_parser(tmp_path,monkeypatch,startup_failure):
    import pus6_motor_driver.j14_process_uart as uart
    import pus6_force_control.manual_recovery as recovery
    real_core=uart.ServoCore;b=StatusBus();created=[]
    request=b.request
    def inject_motion(raw,**kw):
        frame=parse_frame(raw)
        if frame.command==0x32 and frame.register_addr==0x25 and frame.data[:2]==b'\x03\x00':
            if startup_failure=='partial' and frame.motor_id==3:b.positions[3]+=3
            if startup_failure=='barrier' and frame.motor_id==5:b.positions[4]+=4
        return request(raw,**kw)
    b.request=inject_motion
    class Port:
        def close(self):pass
    class Input(io.StringIO):
        def isatty(self):return True
        def fileno(self):return 0
    def factory(config):
        config=dict(config,hardware_mode='mock')
        core=real_core(config,bus=b);core.serial=Port();created.append(core)
        original=core.initialize
        def initialize():
            result=original()
            if startup_failure is True:b.bad='adc'
            return result
        core.initialize=initialize
        return core
    monkeypatch.setattr(uart,'ServoCore',factory)
    # One start space, six per-axis checks, then F before a dynamic command.
    monkeypatch.setattr(recovery.sys,'stdin',Input('       f'))
    monkeypatch.setattr(recovery.termios,'tcgetattr',lambda *_:[])
    monkeypatch.setattr(recovery.termios,'tcsetattr',lambda *_:None)
    monkeypatch.setattr(recovery.tty,'setcbreak',lambda *_:None)
    monkeypatch.setattr(recovery.signal,'signal',lambda *_:None)
    polls=[False]
    def readable(*_):
        polls[0]=not polls[0]
        return ([recovery.sys.stdin] if polls[0] else [],[],[])
    monkeypatch.setattr(recovery.select,'select',readable)
    result=run_manual(dict(execute=True,hardware_write_enabled=True,is_hardware_qualified=True,
        dry_run=False,hardware_mode='active',epoch=1,output_dir=str(tmp_path)))
    assert created[0].closed and result['source']=='MOCK'
    assert not result['hardware_stop_confirmed'] # no real hardware in this test
    if startup_failure is True:
        assert result['outcome']=='stopped' and not result['last_snapshot']['completed']
        assert result['independent_stop_required']
        assert b.write_audit==[]
        assert not result['manual_ready']
    elif startup_failure in ('partial','barrier'):
        assert result['outcome']=='stopped' and not result['manual_ready']
        assert result['terminal_recovery']['position_hold_confirmed']
        assert all(b.modes[i]==0 for i in range(1,7))
        entries=[w for w in b.write_audit if len(w['payload'])==3]
        assert len(entries)==(3 if startup_failure=='partial' else 6)
        assert not any(w['register_address'] in (9,0x27) for w in b.write_audit)
        if startup_failure=='partial':
            assert not any(w['register_address']==0x1A for w in b.write_audit)
        else:
            restored=[w['motor_id'] for w in b.write_audit if len(w['payload'])==5]
            assert restored[0]==4
    else:
        assert result['outcome']=='manual_session_finished'
        assert result['holding_forces_g']==HOLD
        assert result['manual_ready']
        assert len(result['entry_checks'])==24
        assert all(a['mode']==3 for a in result['ready_snapshot']['axes'])
        assert result['terminal_recovery']['position_hold_confirmed']
        assert not result['independent_stop_required']
        entries=[w for w in b.write_audit if w['register_address']==0x25 and len(w['payload'])==3]
        assert len(entries)==6
        assert [w['payload'] for w in entries]==[[3,0,f&65535] for f in HOLD]
        # No full six-axis snapshot between sequential entry writes: each
        # preparation reads only its own real status/mode/force, then mode/target.
        entry_indices=[i for i,event in enumerate(b.events)
            if event[1:3]==(0x32,0x25) and len(event[3])==3]
        assert len(entry_indices)==6
        for left,right in zip(entry_indices,entry_indices[1:]):
            between=b.events[left+1:right]
            assert len(between)==5
            assert all(e[0] in (b.events[left][0],b.events[right][0]) for e in between)
        assert all(b.modes[i]==0 for i in range(1,7))
        assert not any(w['register_address'] in (9,0x1A) for w in b.write_audit)
