import pytest
from core import MockForceBus,TorqueNm,AxialForceN
from position_probe import PositionProbe,PositionStepTarget


def fixture():
    bus=MockForceBus(); bus.positions[1]=1000; bus.modes[1]=0; events=[]
    return bus,PositionProbe(bus,1,2,events.append),events

def test_one_bounded_position_command_and_pause_mock():
    bus,probe,events=fixture(); probe.preflight()
    probe.command(PositionStepTarget(1,1002)); assert probe.read().current_position==1002
    probe.pause_once(); probe.pause_once()
    assert [e[2] for e in bus.events if e[1]==0x32]==[0x29,0x1a]
    assert [e['kind'] for e in events]==['preflight','pre_command_feedback','position_write_attempt','position_write_response','feedback','pause_write_attempt','pause_response']
    assert bus.hardware_write_count==0

def test_refuses_force_torque_other_axis_and_excess_steps():
    bus,probe,events=fixture(); probe.preflight()
    for command in (TorqueNm((1,)*6),AxialForceN((1,)*6)):
        with pytest.raises(TypeError): probe.command(command)
    for command in (PositionStepTarget(2,1001),PositionStepTarget(1,1003),PositionStepTarget(1,1000)):
        with pytest.raises(ValueError): probe.command(command)
    assert not [e for e in bus.events if e[1]==0x32]

def test_wrong_mode_and_fault_block_before_write():
    bus,probe,events=fixture(); bus.modes[1]=1
    with pytest.raises(RuntimeError): probe.preflight()
    assert not [e for e in bus.events if e[1]==0x32]

def test_uncertain_write_is_not_retried_and_pause_is_once():
    bus,probe,events=fixture(); probe.preflight(); bus.fail_on_id=1
    with pytest.raises(OSError): probe.command(PositionStepTarget(1,1001))
    with pytest.raises(RuntimeError): probe.command(PositionStepTarget(1,1001))
    with pytest.raises(OSError): probe.pause_once()
    probe.pause_once()
    assert len([e for e in bus.events if e[2]==0x29])==1
    assert len([e for e in bus.events if e[2]==0x1a])==1
    assert bus.hardware_write_count==0

def test_cli_mock_protocol_lifecycle(monkeypatch,tmp_path):
    import json,sys,types
    import position_probe_cli as cli
    bus=MockForceBus(); bus.positions[1]=1000; bus.modes[1]=0
    monkeypatch.setattr(cli,'check_free',lambda _:None)
    monkeypatch.setattr(cli,'UartBus',lambda **kwargs:bus)
    monkeypatch.setattr(cli.time,'sleep',lambda _:None)
    monkeypatch.setitem(sys.modules,'serial',types.SimpleNamespace(Serial=lambda *args,**kwargs:object()))
    path=tmp_path/'output'
    assert cli.main(['--execute','--axis','1','--delta-steps','2','--max-delta-steps','2',
                     '--observe-s','1','--tolerance-steps','0','--log-dir',str(path)])==0
    result=json.loads((path/'result.json').read_text())
    assert result['initial_steps']==1000 and result['target_steps']==1002
    assert result['target_reached'] and result['pause_acknowledged'] and result['feedback_stable_after_pause']
    assert not result['hardware_stop_verified']
    assert [e[2] for e in bus.events if e[1]==0x32]==[0x29,0x1a]


def test_cli_rejects_motion_without_execute_or_beyond_bound(tmp_path):
    import position_probe_cli as cli
    common=['--axis','1','--delta-steps','3','--max-delta-steps','2','--observe-s','1',
            '--tolerance-steps','0','--log-dir',str(tmp_path/'never')]
    with pytest.raises(SystemExit): cli.main(common)
    with pytest.raises(SystemExit): cli.main(['--execute']+common)
    assert not (tmp_path/'never').exists()


def test_changed_start_or_logging_error_never_prevents_pause():
    bus,probe,events=fixture(); probe.preflight(); bus.positions[1]=1001
    with pytest.raises(RuntimeError): probe.command(PositionStepTarget(1,1002))
    assert not [e for e in bus.events if e[1]==0x32]
    bus.positions[1]=1000; probe.command(PositionStepTarget(1,1001))
    probe.event=lambda event: (_ for _ in ()).throw(OSError('log full'))
    with pytest.raises(OSError): probe.pause_once()
    assert len([e for e in bus.events if e[2]==0x1a])==1
