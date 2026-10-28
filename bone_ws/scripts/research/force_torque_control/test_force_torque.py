import pytest
from core import (Mode, TorqueNm, AxialForceN, MeasuredForce, MockForceBus,
                  MockTorqueBackend, ForceProtocolMock, ForceOuterLoop, ResearchSession, STANDARD_GRAVITY)
from controller import Config, JointTorquePID, Fault

T=1_000_000_000

def make():
    p=JointTorquePID(Config(kp=[10]*6,ki=[1]*6,kd=[0]*6,torque_limit=[.2]*6,
        slew_limit=[1]*6,direction=[1,-1,1,1,1,1],min_dt=.001,max_dt=.2,max_age=.2,derivative_tau=0))
    loop=ForceOuterLoop([100]*6,[10]*6,[100]*6,[1]*6,.2)
    b=MockForceBus(); f=ForceProtocolMock(b,10); t=MockTorqueBackend()
    return ResearchSession(p,loop,t,f),b

def feedback(stamp=T,valid=True,force=1.):
    return MeasuredForce((force,)*6,(stamp,)*6,(valid,)*6,(0,)*6)

def test_torque_pipeline_sign_slew_and_exclusive_mode():
    s,b=make(); s.select(Mode.PID_TORQUE)
    with pytest.raises(RuntimeError): s.select(Mode.ACTUATOR_FORCE_CONTROL)
    assert s.cycle([.1]*6,[0]*6,[T]*6,[0]*6,T) is None
    out=s.cycle([.1]*6,[0]*6,[T+100_000_000]*6,[0]*6,T+100_000_000)
    assert isinstance(out,TorqueNm)
    assert out.values == pytest.approx((.1,-.1,.1,.1,.1,.1))
    assert s.pid.state['error_m'] == pytest.approx([.1]*6)
    assert s.pid.integral == [0]*6
    assert not b.events and b.hardware_write_count == 0
    with pytest.raises(TypeError): s.torque.write(AxialForceN((1,)*6))

def test_force_registers_and_measured_sensor():
    s,b=make(); b.forces_g[1]=-1000
    s.select(Mode.ACTUATOR_FORCE_CONTROL)
    measured=s.force.read_feedback(clock=lambda:T)
    assert measured.values[0] == pytest.approx(-STANDARD_GRAVITY)
    assert measured.valid == (True,)*6
    assert s.cycle([.1]*6,[0]*6,[T]*6,[0]*6,T,measured) is None
    later=T+100_000_000
    out=s.cycle([.1]*6,[0]*6,[later]*6,[0]*6,later,feedback(later))
    assert isinstance(out,AxialForceN)
    assert out.values == pytest.approx([10]*6)
    assert s.force_loop.state['force_error_n'] == pytest.approx([9]*6)
    assert {e[2] for e in b.events} == {0,0x25,0x27}
    assert not any(e[2] == 0x29 for e in b.events)
    with pytest.raises(TypeError): s.force.write(TorqueNm((1,)*6))
    assert b.hardware_write_count == 0

def test_documented_1000g_encoding():
    s,b=make(); s.select(Mode.ACTUATOR_FORCE_CONTROL)
    s.force.write(AxialForceN((STANDARD_GRAVITY,)*6))
    writes=[e for e in b.events if e[2]==0x27]
    assert len(writes)==6 and all(e[3]==[1000] for e in writes)

@pytest.mark.parametrize('sample',[None, feedback(valid=False),feedback(stamp=T-300_000_000),feedback(force=float('nan'))])
def test_invalid_force_feedback_latches_and_mock_pauses(sample):
    s,b=make(); s.select(Mode.ACTUATOR_FORCE_CONTROL)
    with pytest.raises((ValueError,TypeError)):
        s.cycle([0]*6,[0]*6,[T]*6,[0]*6,T,sample)
    assert s.mode is Mode.FAULT
    assert sum(e[2]==0x1a for e in b.events)==6
    s.fail('again')
    assert sum(e[2]==0x1a for e in b.events)==6

def test_stale_position_and_ds_unavailable():
    s,b=make()
    with pytest.raises(RuntimeError): s.select(Mode.DS_SCTDE)
    s.select(Mode.PID_TORQUE)
    with pytest.raises(Fault): s.cycle([0]*6,[0]*6,[T-300_000_000]*6,[0]*6,T)
    assert s.mode is Mode.FAULT and s.torque.stops==1
