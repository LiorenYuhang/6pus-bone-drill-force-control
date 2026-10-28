from dataclasses import replace
import numpy as np
import pytest
from realistic_simulation import CASES, CausalDifference, KIN, Plant, geometry, modeled_wrench, nonlinear_geometry, simulate
from input_availability import audit
from geometry_audit import audit as geometry_audit, sensitivity
from ds_sctde import Config, DSSCTDE, ControllerFault


def test_geometry_against_existing_ik_fk_and_virtual_power():
    rows=geometry_audit()
    assert len(rows)==27 and all(x['fk_converged'] for x in rows)
    assert max(x['jacobian_max_difference_m_per_rad'] for x in rows)<1e-8
    assert max(x['virtual_power_residual'] for x in rows)<1e-12
    assert max(x['fk_residual_m'] for x in rows)<1e-9
    assert max(x['fk_pose_error_norm'] for x in rows)<1e-8


def test_geometry_changes_with_pose():
    q0,j0=nonlinear_geometry([0,0,0]); q1,j1=nonlinear_geometry([.08,-.05,.07])
    assert np.max(abs(j1-j0))>1e-4 and not np.allclose(q0,q1)
    assert np.all(q1>0) and KIN.nominal_height>0
    assert len(sensitivity())==6 and all(x['max_abs_delta_q_m']>0 for x in sensitivity())


def test_ideal_transmission_virtual_power():
    _,j=nonlinear_geometry([.03,-.02,.04]); rate=np.array([.1,.2,-.1])
    n=np.array([.001,-.001,.001,-.001,.001,-.001]); force=np.array([2.,-3.,4.,-2.,1.,-.5])
    tau=n*force; motor_rate=(j@rate)/n
    assert tau@motor_rate == pytest.approx((j.T@force)@rate,abs=1e-12)


def test_near_singular_synthetic_jacobian_fails_closed():
    cfg=Config(np.eye(3),[1]*3,[1]*3,[1]*6,[1]*6,[1]*6,[1]*6,[1]*6,[.001]*6,[1]*6,[1]*6,[1]*6,.001,.1,.2,100.)
    j=np.vstack((np.diag([1.,1.,.001]),np.diag([1.,1.,.001])))
    sample={'timestamp_ns':1_000_000_000,'fault_codes':[0]*6,'eta_rad':[0]*3,'eta_d_rad':[0]*3,
      'eta_dot_rad_s':[0]*3,'eta_dot_d_rad_s':[0]*3,'eta_ddot_rad_s2':[0]*3,'eta_ddot_d_rad_s2':[0]*3,
      'q_m':[.05]*6,'q_d_m':[.05]*6,'q_dot_m_s':[0]*6,'q_dot_d_m_s':[0]*6,'q_ddot_d_m_s2':[0]*6,
      'friction_motor_nm':[0]*6,'jacobian_m_per_rad':j}
    with pytest.raises(ControllerFault,match='conditioning'): DSSCTDE(cfg).update(sample,1_000_000_000)


def test_difference_initialization_and_time_error():
    e=CausalDifference(); assert e.update(1.,np.zeros(3),np.ones(6)*.05) is None
    assert e.update(1.01,np.zeros(3),np.ones(6)*.05) is None
    v=e.update(1.02,np.zeros(3),np.ones(6)*.05)
    assert np.allclose(v[2],0)
    with pytest.raises(ValueError): e.update(1.02,np.zeros(3),np.ones(6)*.05)


def test_current_inputs_fail_closed():
    result=audit(); assert not result['ready'] and result['torque_command'] is None
    assert any('wrench' in x for x in result['reasons']) and result['hardware_write_count']==0


def test_wrench_observer_matches_plant_only_if_assumptions_hold():
    plant=Plant(geometry()); plant.eta=np.array([.01,-.02,.01]); plant.vel=np.array([.1,-.05,.02])
    plant.q=np.array([.0001,-.0002,.0001,0.,.0002,-.0001]); plant.qvel=np.array([.001]*6)
    _,true_w,_=plant.state(0.,0.)
    exact=modeled_wrench(plant.eta,plant.vel,.05+plant.q,plant.qvel,plant.spring,plant.damp)
    wrong=modeled_wrench(plant.eta,plant.vel,.05+plant.q,plant.qvel,.7*plant.spring,plant.damp)
    assert exact == pytest.approx(true_w,abs=1e-12)
    assert np.linalg.norm(wrong-true_w)>1e-5


def test_synthetic_short_simulation_finite():
    sc=replace(CASES[0],duration=.08)
    for kind in ('PID_BASELINE','DS_SCTDE_ORACLE','DS_SCTDE_MODEL_ESTIMATE'):
        rows,summary=simulate(sc,kind)
        assert rows and not summary['fault'] and summary['hardware_write_count']==0
