"""Installed offline entry point; always uses FakeBus and dry-run force frames."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from pus6_motor_driver.fake_bus import FakeBus
from .force_pipeline import ForceCompensation, TaskSpaceForcePipeline
from .six_axis_force_drive import ForceLimitsConfig, SixAxisForceDrive
from .task_space_force_pid import ForcePIDConfig


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    opts = parser.parse_args(args)
    path = opts.config or Path(get_package_share_directory("pus6_force_control")) / "config/force_pid.yaml"
    config = yaml.safe_load(path.read_text())
    if config["frame_id"] != "pus6_base":
        raise ValueError("only pus6_base is supported")
    # A deterministic monotonic timeline for an offline example, not a timing benchmark.
    now = [1_000_000_000]
    clock = lambda: now[0]
    bus = FakeBus(read_only=True)
    bus.positions = {axis: 1000 for axis in range(1, 7)}
    drive = SixAxisForceDrive(bus, ForceLimitsConfig(**config["drive"]), clock=clock)
    pipeline = TaskSpaceForcePipeline(ForcePIDConfig(**config["pid"]), drive=drive,
                                     compensation=ForceCompensation(**config["compensation"]), clock=clock)
    snapshot = drive.read_feedback()
    now[0] += 20_000_000
    result = pipeline.update_from_snapshot(1.0, snapshot)
    assert not bus.write_audit
    print(json.dumps({"source": "MOCK", "hardware_write_count": len(bus.write_audit),
                      "pid": asdict(result.pid), "pose_m_rad": result.pose.tolist(),
                      "measured_wrench_n_nm": result.measured_wrench.tolist(),
                      "predicted_applied_wrench_n_nm": result.applied_wrench.tolist(),
                      "frames_built": len(result.drive_result["frames"])}, indent=2))


if __name__ == "__main__":
    main()
