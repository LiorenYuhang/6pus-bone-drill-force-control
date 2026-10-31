import pytest
from pus6_motor_driver.fake_bus import FakeBus
from uart_driver import SixAxisPositionDrive
from calibration import affine_calibration


def drive():
    bus=FakeBus()
    bus.positions={i:1000 for i in range(1,7)}; bus.targets=bus.positions.copy()
    bus.modes={i:1 for i in range(1,7)}
    events=[]
    d=SixAxisPositionDrive(bus,max_start_delta_steps=10,max_cycle_delta_steps=2,
                           event=lambda kind,data:events.append((kind,data)))
    d.initialize(active=True)
    return d,bus,events


def test_position_conversion_six_axes_feedback_and_pause():
    d,bus,_=drive()
    snapshot=d.command([.050025]*6)
    assert [s.current_position for s in snapshot['status']]==[1001]*6
    assert d.positions_m(snapshot)==pytest.approx([.050025]*6)
    d.pause_once(); d.pause_once()
    writes=[e for e in bus.events if e[1]==0x32]
    assert [e[2] for e in writes]==[0x29]*6+[0x1a]*6
    assert d.position_acknowledgements==6


def test_all_targets_validated_before_first_write():
    d,bus,_=drive()
    with pytest.raises(ValueError): d.command([.05]*5+[.051])
    assert not bus.write_audit


def test_write_error_never_retries_and_attempts_pause_on_all_axes():
    d,bus,_=drive(); bus.fail_on_id=3
    with pytest.raises(OSError): d.command([.050025]*6)
    writes=[e for e in bus.events if e[1]==0x32]
    assert [(e[0],e[2]) for e in writes[:3]]==[(1,0x29),(2,0x29),(3,0x29)]
    assert [(e[0],e[2]) for e in writes[3:]]==[(i,0x1a) for i in range(1,7)]
    assert not any(e[2] in (0x27,0x26) for e in writes)


def test_servo_selection_sets_current_position_before_mode():
    d,bus,_=drive(); bus.events.clear(); bus.modes={i:0 for i in range(1,7)}
    d.modes=[]; d.lower=[]; d.upper=[]
    d.initialize(active=True,select_servo=True)
    writes=[e for e in bus.events if e[1]==0x32]
    assert [(e[2],e[3]) for e in writes]==[(reg,payload) for _ in range(6) for reg,payload in ((0x29,[1000]),(0x25,[1]))]


def test_signed_newton_calibration_and_nonzero_loaded_reference():
    c=affine_calibration([100.]*6,[200.]*6,[1.]*6,[-1.]*6)
    assert c['newtons_per_raw']==pytest.approx([-.02]*6)
    assert c['raw_zero']==pytest.approx([150.]*6)
    with pytest.raises(ValueError): affine_calibration([1.]*6,[1.]*6,[0.]*6,[1.]*6)
