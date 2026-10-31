import pytest
from controller import Config, ForcePositionController


def config(**kw):
    values=dict(kp=(100.,)*6,ki=(0.,)*6,kd=(0.,)*6,force_limit_n=(2.,)*6,
                admittance_m_per_ns=(.001,)*6,correction_limit_m=(.001,)*6,
                correction_rate_m_s=(.01,)*6,raw_zero=(100.,)*6,
                newtons_per_raw=(.01,)*6,max_dt_s=.2,max_age_s=.2)
    values.update(kw)
    return Config(**values)


def step(c,t,q=.01,raw=100,now=None,valid=(True,)*6,faults=(0,)*6):
    return c.update([q]*6,[0.]*6,[raw]*6,[t]*6,valid,faults,t if now is None else now)


def test_signed_calibration_force_error_and_dynamic_position():
    c=ForcePositionController(config())
    assert step(c,1_000_000_000) is None
    assert step(c,1_100_000_000) == pytest.approx([.0101]*6)
    assert c.state['force_cmd_n'] == pytest.approx([1.]*6)
    assert c.state['force_actual_n'] == pytest.approx([0.]*6)
    assert step(c,1_200_000_000,raw=300) == pytest.approx([.01]*6)
    assert c.state['force_error_n'] == pytest.approx([-1.]*6)


def test_force_and_correction_limits_and_antiwindup():
    c=ForcePositionController(config(kp=(1000.,)*6,ki=(10.,)*6,correction_limit_m=(.00005,)*6))
    step(c,1_000_000_000)
    assert step(c,1_100_000_000) == pytest.approx([.01005]*6)
    assert c.integral == [0.]*6
    assert c.state['force_cmd_n'] == [2.]*6


@pytest.mark.parametrize('kwargs',[{'raw':float('nan')},{'now':1_500_000_000},
                                   {'valid':(False,)*6},{'faults':(1,)*6}])
def test_invalid_feedback_latches(kwargs):
    c=ForcePositionController(config()); step(c,1_000_000_000)
    with pytest.raises(RuntimeError): step(c,1_100_000_000,**kwargs)
    with pytest.raises(RuntimeError): step(c,1_200_000_000)
