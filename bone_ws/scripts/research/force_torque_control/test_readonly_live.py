import pytest
from core import MockForceBus
from live_readonly import StatusOnlyBus
from pus6_motor_driver.protocol import pack_read_status_cmd, pack_write_register_cmd, CMD_RD_STATUS
from status_bridge import ReadOnlyStatusSource


def test_readonly_boundary_only_allows_status_queries():
    bus=MockForceBus(); guard=StatusOnlyBus(bus)
    axis,raw,stamp=ReadOnlyStatusSource(guard).poll(1)
    assert axis==1 and stamp>0 and raw
    before=bus.attempt_count
    with pytest.raises(ValueError): guard.request(pack_write_register_cmd(1,0x27,[1]))
    assert bus.attempt_count==before and bus.tx_cmd_counts=={CMD_RD_STATUS:1}
    assert bus.hardware_write_count==0
