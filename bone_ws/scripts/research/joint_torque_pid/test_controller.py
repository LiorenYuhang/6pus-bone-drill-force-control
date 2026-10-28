import math
import pytest
from controller import Config, Fault, JointTorquePID


def cfg(**overrides):
    v = dict(kp=(1.,)*6, ki=(0.,)*6, kd=(0.,)*6, torque_limit=(10.,)*6,
             slew_limit=(1000.,)*6, direction=(1.,)*6, min_dt=.01,
             max_dt=.2, max_age=.2, derivative_tau=0.)
    v.update(overrides)
    return Config(**v)


def update(pid, desired, actual, t, faults=(0,)*6, now=None):
    return pid.update(desired, actual, [t]*6, t if now is None else now, faults)


def test_p_sign_and_independent_axes():
    pid = JointTorquePID(cfg(kp=(1, 2, 3, 4, 5, 6), direction=(1, -1, 1, -1, 1, -1)))
    update(pid, [1]*6, [0]*6, 1_000_000_000)
    assert update(pid, [1]*6, [0]*6, 1_100_000_000) == pytest.approx((1, -2, 3, -4, 5, -6))


def test_integral_derivative_and_state():
    pid = JointTorquePID(cfg(kp=(0,)*6, ki=(2,)*6, kd=(3,)*6))
    update(pid, [0]*6, [0]*6, 1_000_000_000)
    assert update(pid, [.1]*6, [0]*6, 1_100_000_000) == pytest.approx([3.02]*6)
    assert pid.state['i_nm'] == pytest.approx([.02]*6)
    assert pid.state['d_nm'] == pytest.approx([3]*6)


def test_slew_saturation_and_conditional_integration():
    pid = JointTorquePID(cfg(kp=(10,)*6, ki=(1,)*6, torque_limit=(.2,)*6, slew_limit=(1,)*6))
    update(pid, [1]*6, [0]*6, 1_000_000_000)
    assert update(pid, [1]*6, [0]*6, 1_100_000_000) == pytest.approx([.1]*6)
    assert update(pid, [1]*6, [0]*6, 1_200_000_000) == pytest.approx([.2]*6)
    assert pid.integral == [0]*6


@pytest.mark.parametrize('kind', ['nan', 'stale', 'interval', 'fault', 'future'])
def test_fault_latches(kind):
    pid = JointTorquePID(cfg())
    update(pid, [0]*6, [0]*6, 1_000_000_000)
    t = 1_100_000_000
    desired = [math.nan]*6 if kind == 'nan' else [1]*6
    if kind == 'interval': t = 1_000_000_000
    now = t + 300_000_000 if kind == 'stale' else t - 1 if kind == 'future' else t
    with pytest.raises(Fault): update(pid, desired, [0]*6, t, (1,)*6 if kind == 'fault' else (0,)*6, now)
    assert pid.output == [0]*6
    with pytest.raises(Fault): update(pid, [0]*6, [0]*6, 1_150_000_000)


def test_nonfinite_and_invalid_configuration():
    with pytest.raises(ValueError): cfg(kp=(math.inf,)*6)
    with pytest.raises(ValueError): cfg(direction=(0,)*6)
    with pytest.raises(ValueError): cfg(slew_limit=(0,)*6)
