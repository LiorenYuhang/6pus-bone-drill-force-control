import math

import pytest

from .controller import Config, Fault, JointTorqueFeedforwardPID


def config(**kw):
    values = dict(kp=(2.,) * 6, ki=(0.,) * 6, kd=(0.,) * 6,
                  torque_limit=(10.,) * 6, slew_limit=(100.,) * 6,
                  direction=(1., -1., 1., -1., 1., -1.), min_dt=.01,
                  max_dt=.2, max_age=.2, derivative_tau=0.)
    values.update(kw)
    return Config(**values)


def step(pid, t, ff, *, qd=(.1,) * 6, q=(0.,) * 6, ft=None, now=None):
    return pid.update(qd, q, [t] * 6, t if now is None else now,
                      [0] * 6, ff, [t if ft is None else ft] * 6)


def test_sum_and_sign_keep_feedforward_in_motor_coordinates():
    pid = JointTorqueFeedforwardPID(config())
    ff = (1., 2., 3., 4., 5., 6.)
    assert step(pid, 1_000_000_000, ff) == (0.,) * 6
    assert step(pid, 1_100_000_000, ff) == pytest.approx((1.2, 1.8, 3.2, 3.8, 5.2, 5.8))
    assert pid.state['feedforward_nm'] == list(ff)
    assert pid.state['error_m'] == pytest.approx([.1] * 6)


def test_combined_output_limits_slew_and_integral_antiwindup():
    pid = JointTorqueFeedforwardPID(config(kp=(0.,)*6, ki=(1.,)*6,
                 direction=(1.,)*6, torque_limit=(.2,)*6, slew_limit=(1.,)*6))
    step(pid, 1_000_000_000, (1.,)*6, qd=(1.,)*6)
    assert step(pid, 1_100_000_000, (1.,)*6, qd=(1.,)*6) == pytest.approx([.1]*6)
    assert pid.integral == [0.]*6
    assert step(pid, 1_200_000_000, (1.,)*6, qd=(1.,)*6) == pytest.approx([.2]*6)
    assert pid.integral == [0.]*6


def test_feedforward_and_pid_terms_are_summed_before_saturation():
    pid = JointTorqueFeedforwardPID(config(kp=(1.,)*6, ki=(2.,)*6,
                 kd=(3.,)*6, direction=(1.,)*6))
    step(pid, 1_000_000_000, (0.,)*6, qd=(0.,)*6)
    out = step(pid, 1_100_000_000, (.5,)*6, qd=(.1,)*6)
    assert pid.state['p_nm'] == pytest.approx([.1]*6)
    assert pid.state['i_nm'] == pytest.approx([.02]*6)
    assert pid.state['d_nm'] == pytest.approx([3.]*6)
    assert out == pytest.approx([3.62]*6)


@pytest.mark.parametrize('ff,ft', [((math.nan,)*6, 1_100_000_000),
                                    ((0.,)*6, 1), ((0.,)*6, 1_200_000_000)])
def test_bad_feedforward_latches_fault(ff, ft):
    pid = JointTorqueFeedforwardPID(config())
    step(pid, 1_000_000_000, (0.,)*6)
    with pytest.raises(Fault):
        step(pid, 1_100_000_000, ff, ft=ft)
    with pytest.raises(Fault):
        step(pid, 1_200_000_000, (0.,)*6)
