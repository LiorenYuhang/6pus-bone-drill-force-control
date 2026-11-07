"""Mode3 handover/exit with true endpoint feedback, using FakeBus only."""
import pytest
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.position_feedback import position_word_to_steps

Q=[1955,350,122,232,934,2004]
HOLD=[-394,143,-300,-580,247,-480]


def owner(tmp_path,bus=None):
    b=bus or MockServoBus(initial_positions=Q)
    b.modes={i:0 for i in range(1,7)};b.targets={i:min(2000,max(0,p)) for i,p in enumerate(Q,1)}
    b.target_forces={i:-1 for i in range(1,7)};b.actual_forces=dict(enumerate(HOLD,1))
    c=ServoCore(dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path),control_mode='force',
        maintenance_only=True,mode3_only=True,dry_run=False,allow_recovery_feedback_tolerance=True),bus=b)
    c.initialize();assert b.write_audit==[]
    return c,b


def test_live_candidate_replaces_old_minus_one_in_same_entry_frame(tmp_path):
    c,b=owner(tmp_path)
    try:
        state=c.prepare_force_handover()
        assert state['holding_forces_g']==HOLD
        assert [a['position_steps'] for a in state['axes']]==Q
        assert state['axes'][5]['position_raw_u16']==2004
        assert b.write_audit==[]
        c.enter_force_mode_with_handover(state['holding_forces_g'],static_baseline_g=HOLD)
        assert [b.target_forces[i] for i in range(1,7)]==HOLD
        entry=b.write_audit[:]
        assert len(entry)==6
        for i,w in enumerate(entry):
            assert w['register_address']==0x25
            assert w['payload']==[3,0,HOLD[i]&65535]
        result=c.finish_force_control('normal_exit')
        assert result['position_hold_confirmed']
        axis=result['axes'][5]
        assert axis['measured_position_steps']==2004 and axis['position_raw_u16']==2004
        assert axis['anchor_steps']==2000 and axis['endpoint_exit_target_selected']
        assert all(0<=w['payload'][4]<=2000 for w in b.write_audit if len(w['payload'])==5)
        assert not any(w['register_address'] in (9,0x1A,0x29) for w in b.write_audit)
    finally:c.close()


@pytest.mark.parametrize('position',[-5,-1,2004,2005])
def test_signed_feedback_and_endpoint_exit_target_remain_distinct(tmp_path,position):
    c,b=owner(tmp_path);b.positions[3]=position
    try:
        state=c.prepare_force_handover();a=state['axes'][2]
        assert a['position_steps']==position
        assert a['position_raw_u16']==position&65535
        c.enter_force_mode_with_handover(HOLD)
        result=c.finish_force_control('normal_exit')
        a=result['axes'][2]
        assert a['measured_position_steps']==position
        assert a['anchor_steps']==(0 if position<0 else 2000)
        assert result['position_hold_confirmed']
    finally:c.close()


def test_sixth_axis_lost_entry_ack_recovers_with_legal_target_no_pause(tmp_path):
    class Lost(MockServoBus):
        failed=False
        def request(self,raw,**kw):
            f=parse_frame(raw);reply=super().request(raw,**kw)
            if f.command==0x32 and f.motor_id==6 and f.register_addr==0x25 and f.data[:2]==b'\x03\0' and not self.failed:
                self.failed=True;raise TimeoutError('entry executed but ACK lost')
            return reply
    c,b=owner(tmp_path,Lost(initial_positions=Q))
    try:
        with pytest.raises(Exception):c.enter_force_mode_with_handover(HOLD)
        assert [b.modes[i] for i in range(1,7)]==[0]*6
        assert b.targets[6]==2000
        assert not any(w['register_address']==0x1A for w in b.write_audit)
        assert sum(w['register_address']==0x25 and w['motor_id']==6 and w['payload'][0]==3 for w in b.write_audit)==1
    finally:c.close()


def test_feedback_beyond_tolerance_cannot_be_used_as_legal_exit(tmp_path):
    c,b=owner(tmp_path)
    try:
        c.enter_force_mode_with_handover(HOLD);b.positions[6]=2006
        result=c.finish_force_control('normal_exit')
        assert result['independent_stop_required']
        assert result['axes'][5]['anchor_steps'] is None
        assert not any(w['motor_id']==6 and len(w['payload'])==5 for w in b.write_audit)
    finally:c.close()


@pytest.mark.parametrize('position',[-5,-1,2004,2005])
def test_unswitched_mode0_endpoint_target_repaired_only_in_manual_exit(tmp_path,position):
    c,b=owner(tmp_path)
    b.positions[6]=position;b.targets[6]=position
    try:
        def cancel(mid):
            if mid==3:raise RuntimeError('stop mixed entry')
        with pytest.raises(Exception):c.enter_force_mode_with_handover(HOLD,axis_verifier=cancel)
        result=c.force_shutdown_report
        assert result['position_hold_confirmed']
        axis=result['axes'][5]
        assert axis['initial_mode']==0 and axis['measured_position_steps']==position
        assert axis['previous_target_raw_u16']==position&65535
        assert axis['mode0_endpoint_target_repaired']
        assert axis['anchor_steps']==(0 if position<0 else 2000)
        assert not any(w['register_address']==0x1A for w in b.write_audit)
        assert all(0<=w['payload'][4]<=2000 for w in b.write_audit if len(w['payload'])==5)
    finally:c.close()


def test_refresh_each_hold_before_entry_and_keep_final_reference(tmp_path):
    c,b=owner(tmp_path);fresh=[f+7 for f in HOLD]
    prepared=[];ack=[]
    try:
        def prepare(mid):
            prepared.append(mid)
            assert all(b.modes[i]==3 for i in range(1,mid))
            assert all(b.modes[i]==0 for i in range(mid,7))
            return fresh[mid-1]
        c.enter_force_mode_with_handover(HOLD,axis_preparer=prepare,
            axis_status_verifier=lambda mid,status:ack.append((mid,status.current_position)))
        assert prepared==list(range(1,7)) and [i for i,q in ack]==list(range(1,7))
        assert list(c.force_hold_g)==fresh and c.last_command_force_g==fresh
        assert [b.target_forces[i] for i in range(1,7)]==fresh
    finally:c.close()


def test_fresh_out_of_limit_force_rejects_before_that_axis_write(tmp_path):
    c,b=owner(tmp_path)
    try:
        with pytest.raises(Exception,match='force limits'):
            c.enter_force_mode_with_handover(HOLD,axis_preparer=lambda mid:1600 if mid==3 else HOLD[mid-1])
        entries=[w['motor_id'] for w in b.write_audit if len(w['payload'])==3]
        assert entries==[1,2]
        assert [b.modes[i] for i in range(1,7)]==[0]*6
        assert not any(w['register_address']==0x1A for w in b.write_audit)
    finally:c.close()


def test_guard_priority_recovers_moving_axis_first_without_pause(tmp_path):
    c,b=owner(tmp_path)
    def verify(mid,status):
        if mid==5:
            error=RuntimeError('ID4 moving during ID5 entry')
            error.recovery_priority_motor_id=4
            raise error
    try:
        with pytest.raises(Exception):
            c.enter_force_mode_with_handover(HOLD,axis_status_verifier=verify)
        restored=[w['motor_id'] for w in b.write_audit if len(w['payload'])==5]
        assert restored[:5]==[4,1,2,3,5]
        assert not any(w['register_address']==0x1A for w in b.write_audit)
        assert all(b.modes[i]==0 for i in range(1,7))
    finally:c.close()


def test_barrier_failure_can_anchor_affected_axis_first_with_id_order_report(tmp_path):
    c,b=owner(tmp_path)
    try:
        c.enter_force_mode_with_handover(HOLD)
        result=c.finish_force_control('manual_entry_barrier_failed',priority_motor_ids=(4,))
        assert [w['motor_id'] for w in b.write_audit if len(w['payload'])==5]==[4,1,2,3,5,6]
        assert [a['motor_id'] for a in result['axes']]==[1,2,3,4,5,6]
        assert result['position_hold_confirmed']
    finally:c.close()
