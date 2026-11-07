"""Numerical replay of actual Stage 4 records, not simulated hardware evidence."""
import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from pus6_force_control.wrench_allocation import SixAxisWrenchAllocator
from pus6_motor_driver.force_units import g_to_n,n_to_g,quantize_force_g_with_n_limits
from pus6_motor_driver.protocol import parse_frame,unpack_status_response

ROOT=Path(__file__).resolve().parents[4]
DATA=ROOT/'results/stage4_active_stabilization/20261104_153217'
pytestmark=pytest.mark.skipif(not DATA.exists(),reason='recorded Stage 4 experiment not present')


def read(name):return [json.loads(s) for s in (DATA/name).read_text().splitlines()]


@pytest.mark.parametrize('fraction',[0.,.5,1.])
def test_real_pose_independent_jacobian_and_signed_native_force_encoding(fraction):
    data=[r for r in read('command_contexts.jsonl') if 'platform_error_world_rad' in r['diagnostics']
          and abs(r['diagnostics']['pose_error_m_rad'][4])>.03]
    r=data[round(fraction*(len(data)-1))];a=SixAxisWrenchAllocator()
    spec=importlib.util.spec_from_file_location('fd',ROOT/'bone_ws/scripts/analyze_stage3c4_control_chain.py')
    fd=importlib.util.module_from_spec(spec);spec.loader.exec_module(fd)
    j,_=fd.numerical_spatial_jacobian(a.kin,r['pose_m_rad'])
    assert a.compute_analytical_spatial_jacobian(r['pose_m_rad'])==pytest.approx(j,abs=2e-8)
    delta=np.array(r['allocated_delta_force_n'])
    assert j.T@delta==pytest.approx(r['commanded_wrench_n_nm'],abs=1e-8)
    total=np.array(g_to_n(r['hold_g']))+delta
    assert quantize_force_g_with_n_limits(n_to_g(total),[-14.709975]*6,[14.709975]*6)==r['target_force_g']
    assert max(abs(delta))<1.  # property of this recorded short experiment, not a force limit


def test_all_recorded_force_writes_and_statuses_match_contexts_by_cycle_and_motor_id():
    contexts=read('command_contexts.jsonl');states=read('tracking.jsonl')
    frames=[parse_frame(bytes.fromhex(r['raw_hex'])) for r in read('serial_bytes.jsonl') if r['direction']=='TX']
    tx=[f for f in frames if f.command==0x32 and f.register_addr==0x27]
    rx=[r for r in read('rx_frames.jsonl') if r['command']==0x32 and r['register']==0x27]
    assert len(tx)==len(rx)==6*len(contexts)==17820
    for k,(frame,response) in enumerate(zip(tx,rx)):
        cycle,i=divmod(k,6);status=unpack_status_response(bytes.fromhex(response['raw_hex']))
        assert frame.motor_id==response['motor_id']==i+1
        assert int.from_bytes(frame.data,'little',signed=True)==contexts[cycle]['target_force_g'][i]
        assert status.force_g==states[cycle]['force_g'][i]
        assert status.current_position==states[cycle]['positions_steps'][i]
        assert status.current_mA==states[cycle]['currents_ma'][i]


def test_final_reference_keeps_large_inverse_base_component_not_only_half_degree_residual():
    r=read('command_contexts.jsonl')[-1];d=r['diagnostics']
    reference=Rotation.from_matrix(d['reference_rotation_base']).as_euler('xyz',degrees=True)
    command=Rotation.from_matrix(d['command_rotation_base']).as_euler('xyz',degrees=True)
    fk=np.degrees(r['pose_m_rad'][3:])
    assert 10.<reference[1]<13. and 10.<command[1]<13.
    assert abs(fk[1])<.5
    assert d['requested_wrench_n_nm']==pytest.approx(np.array([10,10,10,.2,.2,.2])*d['pose_error_m_rad'])
    assert .035<d['requested_wrench_n_nm'][4]<.045
