"""Unshaped normal Mode 3 force commands; bounded exceptional bias recovery.

Returning a force increment to the measured holding bias is NOT proof of a
mechanical stop. Existing Pause/Mode-0 anchoring remains the terminal action.
"""
import math
import time

from pus6_motor_driver.force_units import g_to_n
from .six_axis_force_drive import ForceLimitsConfig


def dual_imu_force_limits():
    return ForceLimitsConfig(max_rate_g_s=None, brake_rate_g_s=(5000.,)*6)


def recover_holding_bias(*, core, drive, hold, outcome, error, first_cycle_id,
                         read_positions, inventory, ingest, check_positions, record,
                         clock=time.monotonic_ns, sleep=time.sleep):
    """Optional exceptional exit only, never before an urgent hardware stop.

    An IMU-input failure or operator stop can remove the last PID increment at
    5000 g/s when motor feedback is fresh and healthy. Overcurrent, bounds,
    tracking, Mode loss or communication faults skip this ramp and retain the
    existing immediate Pause/Mode-0 recovery. Any uncertain write is not retried.
    """
    result = dict(status='not_required', cycles=0, rate_g_s=list(drive.limits.brake_rate_g_s),
                  motion_stop_confirmed=False, destination='captured_holding_bias_not_zero_force')
    eligible = outcome == 'operator_stopped' or (outcome == 'fault' and 'IMU' in (error or ''))
    if not eligible:
        if outcome == 'fault':result['status'] = 'skipped_urgent_fault'
        return result
    if core.writer.stopped or core.feedback.stopped or not core.mode_3_active:
        result['status'] = 'skipped_owner_stopped_or_mode_unconfirmed'
        return result
    try:
        read_positions()  # original current/fault/force/freshness protection
        check_positions(core.positions)
        if any(axis['mode'] != 3 for axis in inventory()):
            result['status'] = 'skipped_mode3_lost'
            return result
        previous = drive.last_commanded_forces_g
        if previous is None:
            result['status'] = 'skipped_no_acknowledged_force'
            return result
        if all(abs(a-b) <= .5 for a,b in zip(previous,hold)):
            result['status'] = 'holding_bias_already_reached'
            return result
        rate = min(drive.limits.brake_rate_g_s)
        maximum_cycles = int(math.ceil(max(abs(a-b) for a,b in zip(previous,hold))/(rate*.02)))+2
        for offset in range(maximum_cycles):
            start = clock()
            prepared = drive.command_forces_n(g_to_n(hold), dry_run=True, now_ns=start, recovery=True)
            target = [f['target_force_g'] for f in prepared['frames']]
            core.check_hardware_write_permission()
            audit = core.writer.send_force_cycle(target, cycle_id=first_cycle_id+offset, target_timestamp_ns=start)
            result['cycles'] += 1
            ingest(audit.feedbacks, audit.feedback_rx_done_ns, audit.cycle_start_ns)
            check_positions(core.positions)
            record('recovery_force.jsonl', dict(ns=clock(), command_timestamp_ns=start, phase='exceptional_bias_recovery',
                rate_g_s=result['rate_g_s'], target_force_g=target, actual_positions_steps=list(core.positions),
                force_2c_g=list(core.forces_g_raw), currents_ma=list(core.currents), fault_codes=list(core.faults)))
            if target == list(hold):
                result['status'] = 'holding_bias_reached'
                return result
            sleep(max(0., .02-(clock()-start)*1e-9))
        result['status'] = 'bounded_recovery_incomplete'
    except Exception as exc:
        result.update(status='aborted_no_retry', error=f'{type(exc).__name__}: {exc}')
    return result
