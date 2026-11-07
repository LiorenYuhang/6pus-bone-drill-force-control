"""Independent geometry/IK virtual-work checks. No serial or hardware calls."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest
from pus6_force_control.wrench_allocation import SixAxisWrenchAllocator
from pus6_force_control.force_pipeline import TaskSpaceForcePipeline
from pus6_force_control.task_space_force_pid import ForcePIDConfig

path=Path(__file__).resolve().parents[3]/'scripts'/'analyze_stage3c4_control_chain.py'
spec=importlib.util.spec_from_file_location('stage3c4_analysis',path)
analysis=importlib.util.module_from_spec(spec);spec.loader.exec_module(analysis)


@pytest.mark.parametrize('pose',[[0,0,.1326,0,0,0],[.001,-.002,.133,.07,-.05,.03]])
def test_analytical_vs_independent_geometry_and_ik_spatial_fd(pose):
    a=SixAxisWrenchAllocator();jg,ji=analysis.numerical_spatial_jacobian(a.kin,pose)
    assert a.compute_analytical_spatial_jacobian(pose)==pytest.approx(jg,abs=2e-8)
    assert jg==pytest.approx(ji,abs=2e-8)
    # Positive pure Z translation is +q on every rail, not strut extension.
    assert jg[:,2]==pytest.approx(np.ones(6),abs=1e-8)


@pytest.mark.parametrize('axis',range(6))
def test_pure_wrench_independent_numeric_reconstruction_and_finite_work(axis):
    a=SixAxisWrenchAllocator();pose=np.array([.001,-.001,.133,.04,-.03,.02])
    jn,_=analysis.numerical_spatial_jacobian(a.kin,pose)
    w=np.eye(6)[axis];f=a.allocate(w,pose).actuator_forces_n
    assert jn.T@f==pytest.approx(w,abs=1e-7)
    dx=np.array([1e-7,-2e-7,3e-7,1e-6,-2e-6,3e-6])
    pp,rp=analysis.perturb(pose,dx);pm,rm=analysis.perturb(pose,-dx)
    dq=analysis.geometry_q(a.kin,pp[:3],rp)-analysis.geometry_q(a.kin,pm[:3],rm)
    assert float(f@dq)==pytest.approx(float(w@(2*dx)),abs=2e-13)


def test_pid_feedforward_is_not_added_twice_before_allocation():
    p=TaskSpaceForcePipeline(ForcePIDConfig(kp=1),clock=lambda:1_000_000_000)
    hold=[-68,-187,-133,-45,18,-6];p.initialize_hold_forces_g(hold)
    pose=[0,0,p.allocator.kin.nominal_height,0,0,0]
    # J's pure Z column is all ones: one axis +100 g -> +0.980665 N.
    raw=np.array(hold)+[100,0,0,0,0,0]
    r=p.update(2.637517,raw,pose,now_ns=4_000_000_000)
    assert r.pid.output_n==pytest.approx(2*2.637517-.980665)
    assert r.commanded_wrench[2]==r.pid.output_n
    assert r.allocation.task_wrench_ref[2]==r.pid.output_n
    assert r.drive_result['requested_forces_n']==pytest.approx(p.hold_forces_n+r.allocation.actuator_forces_n)


def test_allocation_singularity_uses_dimensionless_not_raw_units():
    # Same dimensionless J for two geometrically scaled mechanisms. Raw mixed
    # units would falsely reject the small one; normalized decision must agree.
    class ScaledAllocator(SixAxisWrenchAllocator):
        def compute_analytical_spatial_jacobian(self,pose):
            return np.diag([1,1,1,self.char_length_m,self.char_length_m,self.char_length_m])
    for length in (.0275,1e-7):
        a=ScaledAllocator(char_length_m=length)
        r=a.allocate(np.zeros(6),[0,0,a.kin.nominal_height,0,0,0])
        assert r.normalized_condition_number==pytest.approx(1)
        assert not r.is_singular
    # Conversely normalized near-singularity must be rejected, even if raw
    # singular values happen to look acceptable in arbitrary coordinate units.
    class IllConditioned(ScaledAllocator):
        def compute_analytical_spatial_jacobian(self,pose):
            return np.diag([1,1,1,1e-6,1e-6,1e-6])
    a=IllConditioned(char_length_m=1e-11)
    with pytest.raises(ValueError,match='normalized_cond'):
        a.allocate(np.zeros(6),[0,0,a.kin.nominal_height,0,0,0])
