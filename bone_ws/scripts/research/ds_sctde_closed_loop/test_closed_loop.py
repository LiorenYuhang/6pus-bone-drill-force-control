from dataclasses import replace
import numpy as np
import pytest
from simulate import CASES, Plant, geometry, simulate


def test_linearized_force_and_virtual_power():
    j=geometry(); p=Plant(j); p.eta=np.array([.01,-.02,.01]); p.q=np.arange(6)*.0001
    f,w,_=p.state(0.,0.)
    assert np.allclose(w,j.T@f)
    eta_rate=np.array([.2,-.1,.04]); q_rate=j@eta_rate
    assert float(f@q_rate)==pytest.approx(float(w@eta_rate),abs=1e-12)


def test_equilibrium_and_finite_repeatability():
    p=Plant(geometry()); p.step(np.zeros(6),0.,.00025,0.)
    assert np.allclose(p.q,0) and np.allclose(p.eta,0)
    sc=replace(CASES[0],duration=.1)
    for mode in ('PID_BASELINE','DS_SCTDE'):
        a,ma=simulate(sc,mode); b,mb=simulate(sc,mode)
        assert ma==mb and a==b and ma['fault']=='' and ma['hardware_write_count']==0


def test_oracle_and_proxy_not_identical_physical_signal():
    p=Plant(geometry()); p.q[:]=.001
    _,w,_=p.state(0.,0.)
    proxy=p.j.T@(np.zeros(6)/p.n)
    assert np.linalg.norm(w-proxy)>1e-6
