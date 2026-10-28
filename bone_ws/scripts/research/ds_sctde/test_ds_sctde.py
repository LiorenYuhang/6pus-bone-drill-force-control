import numpy as np
import pytest
from ds_sctde import Config, DSSCTDE, ControllerFault
from kinematics_map import attitude_jacobian
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics


def config(**kw):
    d = dict(mass=np.eye(3).tolist(), kp=[2.]*3, kd=[3.]*3, cross=[1.]*6, p=[2.]*6,
             kc=[3.]*6, k=[4.]*6, motor_inertia=[.5]*6, transmission=[.01]*6,
             limit=[100.]*6, slew=[10000.]*6, integral_limit=[1.]*6,
             min_dt=.001, max_dt=.2, max_age=.2, max_condition=10000.)
    d.update(kw); return Config(**d)


def sample(t=1_000_000_000):
    return dict(timestamp_ns=t, fault_codes=[0]*6, eta_rad=[0.]*3, eta_d_rad=[0.]*3,
        eta_dot_rad_s=[0.]*3, eta_dot_d_rad_s=[0.]*3, eta_ddot_rad_s2=[1.,2.,3.],
        eta_ddot_d_rad_s2=[0.]*3, q_m=[0.]*6, q_d_m=[0.]*6,
        q_dot_m_s=[0.]*6, q_dot_d_m_s=[0.]*6, q_ddot_d_m_s2=[0.]*6,
        friction_motor_nm=[0.]*6, jacobian_m_per_rad=np.vstack([np.eye(3),np.eye(3)]).tolist(),
        applied_wrench_timestamp_ns=t-100_000_000, applied_task_wrench_prev_nm=[4.,5.,6.])


def test_tde_mapping_matches_explicit_equations():
    c=DSSCTDE(config()); first=sample(); assert c.update(first, first['timestamp_ns']) is None
    s=sample(1_100_000_000); out=c.update(s,s['timestamp_ns'])
    assert c.state['tde_wrench_nm'] == pytest.approx([3.,3.,3.])
    assert c.state['task_wrench_nm'] == pytest.approx([3.,3.,3.])
    assert c.state['actuator_force_n'] == pytest.approx([1.5]*6)
    assert out == pytest.approx([.015]*6)


def test_signed_transmission_controls_motor_direction():
    c=DSSCTDE(config(transmission=[.01,-.01,.01,-.01,.01,-.01]))
    s=sample(); c.update(s,s['timestamp_ns'])
    s=sample(1_100_000_000); result=c.update(s,s['timestamp_ns'])
    assert result == pytest.approx([.015,-.015,.015,-.015,.015,-.015])


def test_cross_coupling_reference_and_six_axis_order():
    c=DSSCTDE(config()); s=sample(); c.update(s,s['timestamp_ns'])
    s=sample(1_100_000_000); s['applied_task_wrench_prev_nm']=[1.,2.,3.]; s['q_d_m']=[1.,0.,0.,0.,0.,0.]
    c.update(s,s['timestamp_ns'])
    assert c.state['cross_deviation_m'] == pytest.approx([2.,-1.,0.,0.,0.,-1.])
    assert c.state['ec_m'] == pytest.approx([1.2,-.1,0,0,0,-.1])
    assert c.state['sigma_m_s'] == pytest.approx([4.4,-1.2,0,0,0,-1.2])
    assert c.state['reference_acceleration_m_s2'] == pytest.approx([4.,-2.,0,0,0,-2.])


def test_kinematics_jacobian_power_mapping():
    kin=Analytical6PusKinematics(); pose=np.array([0.,0.,kin.nominal_height,.01,-.02,.01])
    j=attitude_jacobian(kin,pose); rate=np.array([.2,-.1,.05]); wrench=np.array([.4,-.2,.1])
    force=np.linalg.pinv(j.T)@wrench
    assert j.shape == (6,3)
    assert j.T@force == pytest.approx(wrench,abs=1e-10)
    assert force@(j@rate) == pytest.approx(wrench@rate,abs=1e-10)


def test_sample_rate_and_saturation_integral_freeze():
    c=DSSCTDE(config(limit=[.01]*6,slew=[.1]*6)); s=sample(); c.update(s,s['timestamp_ns'])
    s=sample(1_050_000_000); s['q_d_m']=[1.,0,0,0,0,0]; s['applied_task_wrench_prev_nm']=[1.,2.,3.]
    s['applied_wrench_timestamp_ns']=1_000_000_000
    out=c.update(s,s['timestamp_ns']); assert max(abs(x) for x in out) <= .005000001
    assert np.all(c.integral_c == 0) and c.state['saturated']


@pytest.mark.parametrize('kind',['stale','nan','fault','dt','wrench_time','rank','inf'])
def test_bad_inputs_latch(kind):
    c=DSSCTDE(config()); a=sample(); c.update(a,a['timestamp_ns']); s=sample(1_100_000_000)
    if kind=='nan': s['q_m'][2]=float('nan')
    if kind=='inf': s['eta_ddot_rad_s2'][0]=float('inf')
    if kind=='fault': s['fault_codes'][2]=1
    if kind=='dt': s['timestamp_ns']=1_000_000_000
    if kind=='wrench_time': s['applied_wrench_timestamp_ns']=1
    if kind=='rank': s['jacobian_m_per_rad']=[[0.]*3]*6
    now=s['timestamp_ns'] + (300_000_000 if kind=='stale' else 0)
    with pytest.raises(ControllerFault): c.update(s,now)
    assert np.all(c.output == 0)
    with pytest.raises(ControllerFault): c.update(sample(1_100_000_000),1_100_000_000)
    c.reset(); assert c.fault is None


def test_noise_and_delay_not_hidden():
    c=DSSCTDE(config()); a=sample(); c.update(a,a['timestamp_ns'])
    b=sample(1_100_000_000); b['eta_ddot_rad_s2']=[100.,2.,3.]
    c.update(b,b['timestamp_ns'])
    d=sample(1_200_000_000); d['applied_wrench_timestamp_ns']=1_100_000_000
    c.update(d,d['timestamp_ns'])
    assert c.state['tde_wrench_nm'][0] == pytest.approx(-96.)


def test_same_input_diagnostic_comparison():
    from offline_compare import run
    result=run()
    assert result['steps']==100 and result['dt_s']==.02
    assert result['hardware_write_count']==0
    assert np.isfinite(result['pid_output_rms_nm']) and np.isfinite(result['ds_output_rms_nm'])
