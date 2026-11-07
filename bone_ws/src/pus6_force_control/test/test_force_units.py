"""Mixed-unit boundaries, offline only: no serial port and no hardware writes."""
import math
import numpy as np
import pytest

from pus6_motor_driver.force_units import (
    NOMINAL_N_PER_G, DEFAULT_FORCE_LIMIT_N, g_to_n, n_to_g,
    quantize_force_g, encode_signed_i16_to_u16, decode_u16_to_signed_i16,
)
from pus6_motor_driver.protocol import parse_frame
from pus6_force_control.force_pipeline import ForceCompensation, TaskSpaceForcePipeline
from pus6_force_control.six_axis_force_drive import SixAxisForceDrive, ForceLimitsConfig
from pus6_force_control.task_space_force_pid import ForcePIDConfig


@pytest.mark.parametrize('raw,expected', [(1000,9.80665),(100,.980665),(-1000,-9.80665)])
def test_nominal_g_to_n(raw, expected):
    assert g_to_n(raw) == pytest.approx(expected)
    assert n_to_g(expected) == pytest.approx(raw)


def test_six_axis_roundtrip_direction_and_calibration():
    force_n = np.array([-10., 1., -2., 3., -4., 0.])
    assert g_to_n(n_to_g(force_n)) == pytest.approx(force_n)
    raw = np.array([-1000., 100., -200., 300., -400., 0.])
    assert np.sign(g_to_n(raw)).tolist() == np.sign(raw).tolist()
    directions = [1,-1,1,-1,1,-1]
    scales = [.01,.02,.03,.04,.05,.06]
    assert n_to_g(g_to_n(raw,directions,scales),directions,scales) == pytest.approx(raw)
    # Integer g quantization has at most half a native g of nominal error.
    restored = g_to_n(quantize_force_g(n_to_g(force_n)))
    assert np.abs(restored-force_n).max() <= NOMINAL_N_PER_G/2


@pytest.mark.parametrize('raw', [-32768,32767,-1000,0,1000])
def test_int16_endpoints_and_signed_encoding(raw):
    assert decode_u16_to_signed_i16(encode_signed_i16_to_u16(quantize_force_g(raw))) == raw
    assert quantize_force_g(n_to_g(g_to_n(raw))) == raw


@pytest.mark.parametrize('bad', [32768,-32769,32767.1,-32768.1,float('nan'),float('inf')])
def test_overflow_or_nonfinite_does_not_wrap(bad):
    with pytest.raises(ValueError):quantize_force_g(bad)


def test_rounding_and_invalid_calibration():
    assert quantize_force_g([1.49,1.5,2.5,-1.49,-1.5,-2.5]) == [1,2,2,-1,-2,-2]
    for kwargs in ({'direction':0},{'scale_n_per_g':0},{'scale_n_per_g':float('nan')}):
        with pytest.raises(ValueError):g_to_n(100,**kwargs)
    with pytest.raises(ValueError):g_to_n([1,2])


@pytest.mark.parametrize('sign', [1,-1])
def test_jacobian_pid_allocation_and_register_units_once(sign):
    now=[1_000_000_000]
    drive=SixAxisForceDrive(None,ForceLimitsConfig(max_rate_g_s=(1e9,)*6),clock=lambda:now[0])
    pipe=TaskSpaceForcePipeline(ForcePIDConfig(kp=0,reference_rate_n_s=1000),
                               drive=drive,clock=lambda:now[0])
    pose=[0,0,pipe.allocator.kin.nominal_height,0,0,0]
    force_g_raw=[sign*100]*6
    now[0]+=1_000_000_000
    cycle=pipe.update(sign*6*.980665,force_g_raw,pose)
    J=cycle.allocation.jacobian_6x6
    assert cycle.measured_wrench == pytest.approx(J.T @ np.array(g_to_n(force_g_raw)))
    assert cycle.pid.measured_n == pytest.approx(sign*6*.980665)
    assert cycle.allocation.actuator_forces_n == pytest.approx([sign*.980665]*6)
    assert pipe.allocator.reconstruct_task_wrench_n(cycle.allocation.actuator_forces_n,pose) == pytest.approx(cycle.commanded_wrench, abs=1e-9)
    assert cycle.drive_result['dispatched_forces_g'] == pytest.approx([sign*100]*6)
    for frame in cycle.drive_result['frames']:
        decoded=parse_frame(bytes.fromhex(frame['frame_hex']))
        assert decoded.register_addr == 0x27
        assert decode_u16_to_signed_i16(int.from_bytes(decoded.data,'little')) == sign*100
    assert cycle.applied_wrench == pytest.approx(cycle.commanded_wrench, abs=1e-9)
    assert cycle.drive_result['dry_run']


def test_per_axis_n_limits_precede_native_g_s_limiter():
    now=[1_000_000_000]
    caps_n=tuple(g_to_n([100,200,300,400,500,600]))
    limits=ForceLimitsConfig(min_force_n=tuple(-v for v in caps_n),max_force_n=caps_n)
    drive=SixAxisForceDrive(None,limits,clock=lambda:now[0])
    drive.last_commanded_forces_g=[0]*6;drive.last_command_time_ns=now[0]
    now[0]+=20_000_000
    result=drive.command_forces_n([10,-10,10,-10,10,-10])
    assert result['limited_forces_n'] == pytest.approx([caps_n[i]*(-1 if i%2 else 1) for i in range(6)])
    assert result['target_forces_g'] == pytest.approx([100,-200,300,-400,500,-600])
    assert result['dispatched_forces_g'] == pytest.approx([10,-10,10,-10,10,-10]) # 500 g/s * .020 s
    assert DEFAULT_FORCE_LIMIT_N == pytest.approx(14.709975)


def test_calibration_is_shared_by_feedback_caps_and_commands():
    now=[1_000_000_000]
    comp=ForceCompensation(newtons_per_g=(.02,)*6,direction=(-1,1,-1,1,-1,1))
    drive=SixAxisForceDrive(None,ForceLimitsConfig(min_force_n=(-1,)*6,max_force_n=(1,)*6,
                           max_rate_g_s=(1e9,)*6),clock=lambda:now[0])
    pipe=TaskSpaceForcePipeline(ForcePIDConfig(kp=0,reference_rate_n_s=1000),
                               compensation=comp,drive=drive,clock=lambda:now[0])
    now[0]+=1_000_000_000
    cycle=pipe.update(10,[0]*6,[0,0,pipe.allocator.kin.nominal_height,0,0,0])
    assert cycle.drive_result['limited_forces_n'] == pytest.approx([1]*6)
    assert [r['target_force_g'] for r in cycle.drive_result['frames']] == [-50,50,-50,50,-50,50]
    assert cycle.applied_wrench[2] == pytest.approx(6)
    assert cycle.actuator_limited


def test_child_si_caps_and_nondefault_scale_before_native_write(tmp_path):
    from pus6_motor_driver.j14_process_uart import ServoCore
    from pus6_motor_driver.fake_bus import MockServoBus
    bus=MockServoBus()
    config=dict(epoch=1,output_dir=str(tmp_path),hardware_mode='mock',dry_run=True,
                control_mode='force',initial_force_g=[0]*6,newtons_per_g=[.02]*6,
                direction=[1,-1,1,-1,1,-1],min_force_n=-.1,max_force_n=.1,max_rate_g_s=500.)
    core=ServoCore(config,bus=bus)
    try:
        core.initialize()
        core.command.candidate_force=lambda *args:[1,-1,1,-1,1,-1]
        packet=core.cycle(0)
        assert core.last_command_force_g == [5]*6
        assert packet['dispatched_forces_n'] == pytest.approx([.1,-.1,.1,-.1,.1,-.1])
        assert not bus.write_audit
    finally:
        core.writer.latch_fault_without_pause('offline test complete')
        core.close();core.rec._dumped=True


def test_historical_tare_schema_read_is_not_live_alias():
    from pus6_force_control.six_axis_tare_experiment import normalize_archived_units
    from pus6_force_control.hardware_tare import AxisTareState
    archived={'before_force_gf':-100,'after_forces_gf':[0,1]}
    native=normalize_archived_units(archived)
    assert native == {'before_force_g':-100,'after_forces_g':[0,1]}
    assert archived['before_force_gf'] == -100
    assert not hasattr(AxisTareState(1),'before_force_gf')


@pytest.mark.parametrize('direction', [1,-1])
def test_integer_encoding_does_not_round_past_n_caps(direction):
    from pus6_motor_driver.force_units import quantize_force_g_with_n_limits
    caps_n = .985  # nearest integer g would exceed this cap
    encoded = quantize_force_g_with_n_limits(n_to_g([caps_n]*6,direction),
                                            -caps_n,caps_n,direction)
    assert max(g_to_n(encoded,direction)) <= caps_n
    assert abs(encoded[0]) == 100
