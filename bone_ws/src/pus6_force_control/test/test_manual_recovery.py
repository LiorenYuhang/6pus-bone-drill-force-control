"""No serial access: manual force math, stroke handling and measured FK/IK only."""
import numpy as np
import pytest
from pus6_force_control.manual_recovery import (
    ManualCompliance, ManualRecoveryConfig, recovery_plan, run_manual, validate_positions)
from pus6_motor_driver.force_units import g_to_n


HOLD=[100,-200,300,-400,500,-600]


def test_real_reported_recovery_feedback_is_preserved_and_commands_separate():
    from pus6_motor_driver.position_feedback import recovery_exit_target_steps, validate_position_target_steps
    q=[1955,350,122,232,934,2004]
    r=recovery_plan(q)
    assert r['input_steps']==q and r['recovery_ready']
    assert r['exit_target_steps_if_unchanged']==[1955,350,122,232,934,2000]
    assert not r['automatic_motion'] and r['hardware_write_count']==0
    assert validate_positions([-5,-1,0,2000,2004,2005]).tolist()==[-5,-1,0,2000,2004,2005]
    assert [recovery_exit_target_steps(x) for x in (-5,-1,2004,2005)]==[0,0,2000,2000]
    for bad in (65535,-6,2006):
        with pytest.raises(ValueError):validate_positions([1000,1000,1000,bad,1000,1000])
    for bad in (-5,-1,2004,2005):
        with pytest.raises(ValueError):validate_position_target_steps(bad)


def test_measured_fk_and_coordinated_ik_use_actual_seed_not_home():
    from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
    k=Analytical6PusKinematics();pose=[.0002,-.0001,k.nominal_height+.001,.001,-.001,.001]
    q=np.rint((np.asarray(k.inverse_kinematics(pose))-.05)/.000025+1000).astype(int).tolist()
    r=recovery_plan(q)
    assert r['fk_converged'] and r['recovery_ready']
    assert r['candidate_coordinated_path_steps'][0]==pytest.approx(q,abs=1)
    assert r['candidate_coordinated_path_steps'][-1]==pytest.approx([1000]*6,abs=1)
    assert not r['candidate_path_is_hardware_qualified']


def test_signed_hold_preserved_and_damping_opposes_motion_in_si():
    p=ManualCompliance([1000]*6,HOLD,1_000_000_000,ManualRecoveryConfig(control_strategy='velocity_damping'))
    r=p.update([1000]*6,HOLD,[0]*6,[0]*6,now_ns=1_020_000_000,operator_ns=1_020_000_000)
    assert r['target_force_g']==HOLD
    r=p.update([1001,999,1000,1000,1000,1000],HOLD,[0]*6,[0]*6,
        now_ns=1_040_000_000,operator_ns=1_040_000_000)
    assert r['delta_force_n'][0]<0 and r['delta_force_n'][1]>0
    assert np.max(abs(np.asarray(r['target_force_g'])-HOLD))<=10 # 500 g/s * .02 s
    assert r['hold_force_g']==HOLD
    assert r['target_force_n']==pytest.approx(np.array(g_to_n(HOLD))+r['delta_force_n'])


@pytest.mark.parametrize('failure',['operator','gap','speed','current','fault','force'])
def test_manual_session_aborts_bad_input_instead_of_leaving_constant_force(failure):
    p=ManualCompliance([1000]*6,HOLD,1_000_000_000)
    q=[1000]*6;force=HOLD[:];current=[0]*6;fault=[0]*6;now=1_020_000_000;op=now
    if failure=='operator':op=0
    if failure=='gap':now=1_300_000_000;op=now
    if failure=='speed':q[0]=1003
    if failure=='current':current[1]=201
    if failure=='fault':fault[2]=1
    if failure=='force':force[3]=1600
    with pytest.raises(RuntimeError):p.update(q,force,current,fault,now_ns=now,operator_ns=op)


def test_end_recovery_allows_inward_only_without_expanding_absolute_stroke():
    p=ManualCompliance([1955,354,122,231,934,1995],HOLD,1_000_000_000)
    q=[1954,354,123,231,934,1994]
    p.update(q,HOLD,[0]*6,[0]*6,now_ns=1_020_000_000,operator_ns=1_020_000_000)
    # Slow outward movement can still cross the end-zone guard; not a speed fault.
    with pytest.raises(RuntimeError,match='outward'):
        p.update([1957,354,123,231,934,1994],HOLD,[0]*6,[0]*6,
                 now_ns=1_120_000_000,operator_ns=1_120_000_000)


def test_real_execution_permissions_checked_before_device_constructor():
    with pytest.raises(ValueError,match='authorization'):run_manual(dict(execute=True))


@pytest.mark.parametrize('position',[2004,2005,-1,-5])
def test_endpoint_feedback_can_start_and_does_not_add_outward_force(position):
    initial=[1955,350,122,232,934,position]
    hold=[-394,143,-300,-580,247,-480]
    p=ManualCompliance(initial,hold,1_000_000_000)
    q=initial[:];q[0]-=1;q[2]+=1
    q[5]+= -1 if position>=1800 else 1
    r=p.update(q,hold,[0]*6,[0]*6,now_ns=1_020_000_000,operator_ns=1_020_000_000)
    assert r['measured_position_steps']==q
    assert r['target_force_g']==hold # inward manual movement adds no outward damping target
    assert p.initial.tolist()==initial


def test_end_guard_rejects_slow_cumulative_outward_drift():
    p=ManualCompliance([1955,350,122,232,934,2004],HOLD,1_000_000_000)
    for step in (1,2):
        q=[1955+step,350,122,232,934,2004]
        now=1_000_000_000+step*20_000_000
        r=p.update(q,HOLD,[0]*6,[0]*6,now_ns=now,operator_ns=now)
        assert r['target_force_g'][0]<=HOLD[0]
    with pytest.raises(RuntimeError,match='outward'):
        p.update([1958,350,122,232,934,2004],HOLD,[0]*6,[0]*6,
                 now_ns=1_060_000_000,operator_ns=1_060_000_000)


def test_native_support_has_no_artificial_drag_or_position_stiffness():
    p=ManualCompliance([1000]*6,HOLD,1_000_000_000)
    for i in range(1,5):
        now=1_000_000_000+i*50_000_000
        r=p.update([1000+i,1000-i,1000,1000,1000,1000],HOLD,[0]*6,[0]*6,
                   now_ns=now,operator_ns=now)
        assert r['control_strategy']=='native_support'
        assert r['target_force_g']==HOLD
        assert r['delta_force_n']==[0]*6
        assert r['velocity_m_s'][0]>0 and r['velocity_m_s'][1]<0


def test_velocity_uses_per_axis_receive_times_not_snapshot_end():
    p=ManualCompliance([1000]*6,HOLD,1_100_000_000,
        position_ns=[1_000_000_000+i*10_000_000 for i in range(6)])
    r=p.update([1001]*6,HOLD,[0]*6,[0]*6,now_ns=1_160_000_000,operator_ns=1_160_000_000,
        position_ns=[1_050_000_000+i*12_000_000 for i in range(6)])
    assert r['velocity_m_s'][0]==pytest.approx(.000025/(.08+.05))
    assert r['velocity_m_s'][5]==pytest.approx(.000025/(.08+.06))
    with pytest.raises(RuntimeError,match='duplicate'):
        p.update([1001]*6,HOLD,[0]*6,[0]*6,now_ns=1_170_000_000,operator_ns=1_170_000_000,
            position_ns=r['position_source_ns'])


def test_historical_trigger_retains_two_step_limit_and_identifies_axis():
    from pus6_force_control.manual_recovery import ManualHandoverGuard
    guard=ManualHandoverGuard([1955,350,121,232,933,2004],ManualRecoveryConfig())
    # Actual recorded ID4 snapshot: current below limit, displacement above it.
    with pytest.raises(Exception,match='movement/current') as error:
        guard.check_values(4,position_steps=236,force_g=-978,force_adc=1900,
                           current_ma=29,fault_code=0,phase='historical_snapshot')
    assert error.value.recovery_priority_motor_id==4
