"""Exercise the single executable, actual DDS, synthetic status and cleanup."""
import json
import os
from pathlib import Path
import subprocess
import sys
import pytest

HERE=Path(__file__).resolve().parent
ENTRY=HERE/'research_entry.py'
CONFIG=HERE/'example_mock_config.json'

def run(tmp_path,*args):
    env=dict(os.environ, ROS_DOMAIN_ID='173')
    return subprocess.run([sys.executable,str(ENTRY),'--config',str(CONFIG),'--log-dir',str(tmp_path/'run'),*args],
                          env=env,capture_output=True,text=True,timeout=20)

def events(tmp_path):
    return [json.loads(line) for line in (tmp_path/'run'/'telemetry.jsonl').read_text().splitlines()]

@pytest.mark.parametrize('mode,kind',[('PID_TORQUE','torque'),('ACTUATOR_FORCE_CONTROL','force')])
def test_unified_synthetic_runtime(mode,kind,tmp_path):
    result=run(tmp_path,'--mode',mode,'--duration','1')
    assert result.returncode==0, result.stdout+'\n'+result.stderr
    records=events(tmp_path)
    assert any(r['kind']==kind for r in records)
    assert any(r['kind']=='imu' and len(r['data']['ik_lengths_m'])==6 for r in records)
    controls=[r['data'] for r in records if r['kind']=='control' and isinstance(r['data'],dict)]
    assert controls and all(d['actual_m']==pytest.approx([.05]*6) for d in controls)
    assert all(all(t>0 for t in d['position_rx_ns']) for d in controls)
    assert all(d['hardware_write_count']==0 for d in controls)
    assert records[-1]['kind']=='stop' and records[-1]['data']['mode']=='STOPPED'
    assert records[-1]['data']['hardware_write_count']==0
    assert records[-1]['data']['counts'][kind]>0

def test_missing_external_inputs_fault_and_stop(tmp_path):
    result=run(tmp_path,'--input','external','--duration','2')
    assert result.returncode==2, result.stdout+'\n'+result.stderr
    records=events(tmp_path)
    assert records[-1]['kind']=='stop' and records[-1]['data']['hardware_write_count']==0
    assert 'missing' in records[-1]['data']['reason']

def test_real_backend_rejected_before_start(tmp_path):
    config=json.loads(CONFIG.read_text()); config['backend']='real'
    path=tmp_path/'config.json'; path.write_text(json.dumps(config))
    result=subprocess.run([sys.executable,str(ENTRY),'--config',str(path),'--log-dir',str(tmp_path/'run')],capture_output=True,text=True,timeout=10)
    assert result.returncode != 0 and 'real backend unavailable' in result.stderr
    assert not (tmp_path/'run').exists()

def test_mixed_synthetic_hardware_provenance_rejected(tmp_path):
    result=run(tmp_path,'--status-source','hardware','--duration','1')
    assert result.returncode != 0 and 'invalid status provenance' in result.stderr
    assert not (tmp_path/'run').exists()


def test_hardware_force_unit_not_assumed(tmp_path):
    result=run(tmp_path,'--input','external','--status-source','hardware','--duration','1')
    assert result.returncode != 0 and 'unit conversion must be explicitly confirmed' in result.stderr
    assert not (tmp_path/'run').exists()


def test_interrupt_cleans_up_mock(tmp_path):
    import signal
    import time
    env=dict(os.environ, ROS_DOMAIN_ID='174')
    proc=subprocess.Popen([sys.executable,str(ENTRY),'--config',str(CONFIG),'--log-dir',str(tmp_path/'run')],
                          env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        deadline=time.monotonic()+8
        log=tmp_path/'run'/'telemetry.jsonl'
        while time.monotonic()<deadline and (not log.exists() or '"kind": "force"' not in log.read_text()):
            if proc.poll() is not None: break
            time.sleep(.03)
        assert proc.poll() is None and log.exists()
        proc.send_signal(signal.SIGINT)
        stdout,stderr=proc.communicate(timeout=8)
        assert proc.returncode==0,stdout+'\n'+stderr
        records=events(tmp_path)
        assert records[-1]['kind']=='stop' and records[-1]['data']['mode']=='STOPPED'
    finally:
        if proc.poll() is None: proc.kill(); proc.wait()
