"""Bounded terminal handling on the existing UART owner, never HOME or Tare.

A Pause status ACK is not a Mode 3 stop-completion certificate. The stronger
terminal condition is fresh Mode 0 + current-position anchor readback, followed
by a short observed stationary window. SIGKILL/USB loss still require independent
hardware stopping. All writes below use the transport's zero-retry primitives.
"""
import time
from .motor import Motor
from .position_feedback import position_word_to_steps, validate_feedback_position_steps
from .safety import FEEDBACK_POSITION_TOLERANCE_STEPS


def finish_force_control(writer, *, reason, pause=None, pause_first=False,
                         allow_pause_fallback=True, observation_s=.02,
                         max_current_ma=500, skip_anchor_ids=(), allow_feedback_tolerance_anchors=False,
                         priority_motor_ids=(), reanchor_mode0=False):
    priority=tuple(priority_motor_ids)
    if len(set(priority))!=len(priority) or any(type(mid) is not int or mid not in range(1,7) for mid in priority):
        raise ValueError('terminal priority requires unique motor IDs 1..6')
    report = dict(reason=str(reason), axes=[], pause_requested=False,
                  pause_acknowledged=False, position_hold_confirmed=False,
                  independent_stop_required=True, hardware_stop_confirmed=False)
    writer.latch_fault_without_pause(reason)  # reject every subsequent normal command

    def request_pause():
        if pause is None or report['pause_requested']:
            return
        report['pause_requested'] = True
        try:
            audit = pause(reason)
            report['pause_acknowledged'] = bool(audit.success)
            report['pause_axes'] = [dict(motor_id=r.motor_id,
                acknowledged=bool(r.response_received and r.status is not None
                                  and not r.status.error_code), error=r.error)
                for r in audit.records]
        except Exception as exc:
            report['pause_error'] = str(exc)
        # Mode-specific physical Pause behavior is not established by this ACK.

    with writer.uart_lock:
        if pause_first:
            request_pause()
        try:
            writer.drain_residual()
        except Exception as exc:
            report['drain_error'] = str(exc)
        for mid in priority+tuple(mid for mid in range(1,7) if mid not in priority):
            axis = dict(motor_id=mid, final_mode=None, anchor_steps=None,
                        anchor_confirmed=False, observation_confirmed=False)
            report['axes'].append(axis)
            try:
                mode = writer._read_register_locked(mid, 0x25)
                axis['initial_mode'] = mode
                raw = writer._read_register_locked(mid, 0x2A)
                axis['position_raw_u16'] = raw
                # Preserve the raw word and signed measurement separately.
                # Endpoint-tolerance exits require the manual owner's opt-in;
                # ordinary control keeps its strict command-stroke contract.
                measured=position_word_to_steps(raw)
                axis['measured_position_steps']=measured
                if allow_feedback_tolerance_anchors:
                    validate_feedback_position_steps(measured)
                elif not 0 <= measured <= 2000:
                    raise ValueError(f'ID{mid} invalid/out-of-range position {raw:#06x}')
                if mode == 3 and mid in skip_anchor_ids:
                    raise RuntimeError('previous anchor write was uncertain/failed; no automatic retry')
                if mode == 3:
                    axis['anchor_steps'] = writer._anchor_axis_to_position_mode(mid, measured,
                        allow_feedback_tolerance=allow_feedback_tolerance_anchors)
                    axis['endpoint_exit_target_selected'] = axis['anchor_steps'] != measured
                elif mode == 0:
                    # Normally leave un-switched axes untouched. Manual recovery
                    # may repair a pre-existing endpoint target outside the legal
                    # command stroke, never by falsifying the actual position.
                    axis['anchor_steps'] = writer._read_register_locked(mid, 0x29)
                    if reanchor_mode0:
                        axis['previous_target_raw_u16'] = axis['anchor_steps']
                        axis['anchor_steps'] = writer._anchor_axis_to_position_mode(mid, measured,
                            allow_feedback_tolerance=allow_feedback_tolerance_anchors)
                        axis['mode0_current_position_reanchored'] = True
                        axis['endpoint_exit_target_selected'] = axis['anchor_steps'] != measured
                    elif not 0 <= axis['anchor_steps'] <= 2000:
                        axis['previous_target_raw_u16'] = axis['anchor_steps']
                        old_target = position_word_to_steps(axis['anchor_steps'])
                        if not allow_feedback_tolerance_anchors:
                            raise ValueError('invalid existing position target')
                        validate_feedback_position_steps(old_target)
                        if 0 <= measured <= 2000 or abs(old_target-measured)>5:
                            raise ValueError('invalid existing target is not a matching endpoint')
                        axis['anchor_steps'] = writer._anchor_axis_to_position_mode(mid, measured,
                            allow_feedback_tolerance=True)
                        axis['mode0_endpoint_target_repaired'] = True
                        axis['endpoint_exit_target_selected'] = axis['anchor_steps'] != measured
                else:
                    raise ValueError(f'unsupported mode {mode}; no guessed transition')
                axis['final_mode'] = writer._read_register_locked(mid, 0x25)
                axis['target_position_steps'] = writer._read_register_locked(mid, 0x29)
                axis['anchor_confirmed'] = (axis['final_mode'] == 0 and
                    axis['target_position_steps'] == axis['anchor_steps'])
            except Exception as exc:
                axis['error'] = f'{type(exc).__name__}: {exc}'
                # A lost write ACK is delivery-uncertain. Read-only inventory,
                # never retry that write and never declare an unknown axis 0.
                try:
                    axis['final_mode'] = writer._read_register_locked(mid, 0x25)
                except Exception:
                    pass
        report['axes'].sort(key=lambda axis:axis['motor_id'])
        if allow_pause_fallback and not all(a['anchor_confirmed'] for a in report['axes']):
            request_pause()
        for axis in report['axes']:
            samples = []
            try:
                for _ in range(3):
                    if observation_s:
                        time.sleep(observation_s)
                    status = Motor(writer.bus, axis['motor_id']).read_status()
                    mode = writer._read_register_locked(axis['motor_id'], 0x25)
                    axis['final_mode'] = mode
                    samples.append(dict(position_steps=status.current_position,
                        target_position_steps=status.target_position,
                        current_ma=status.current_mA, fault_code=status.error_code, mode=mode))
                axis['observations'] = samples
                positions = [r['position_steps'] for r in samples]
                allowed_error=FEEDBACK_POSITION_TOLERANCE_STEPS if allow_feedback_tolerance_anchors else 2
                def observed_position_valid(position):
                    if allow_feedback_tolerance_anchors:
                        try:validate_feedback_position_steps(position);return True
                        except ValueError:return False
                    return 0<=position<=2000
                axis['observation_confirmed'] = bool(axis['anchor_confirmed'] and
                    max(positions)-min(positions) <= 1 and
                    all(observed_position_valid(r['position_steps']) and r['mode'] == 0
                        and r['target_position_steps'] == axis['anchor_steps']
                        and abs(r['position_steps']-axis['anchor_steps']) <= allowed_error
                        and abs(r['current_ma']) <= max_current_ma and not r['fault_code']
                        for r in samples))
            except Exception as exc:
                axis['observation_error'] = str(exc)
        confirmed = all(a['observation_confirmed'] for a in report['axes'])
        if not confirmed and allow_pause_fallback:
            request_pause()
        report.update(position_hold_confirmed=confirmed, hardware_stop_confirmed=confirmed,
                      independent_stop_required=not confirmed,
                      status='MODE0_ANCHORED_OBSERVED' if confirmed else 'STOP_UNCONFIRMED')
    return report
