"""Separate signed position measurements from legal position command targets.

Endpoint feedback tolerance never enlarges the command stroke. A recovery exit
may explicitly select the nearest legal target; the measurement remains intact.
"""
import math
import numbers
try:
    from .safety import (ABSOLUTE_RAW_LOWER_LIMIT, ABSOLUTE_RAW_UPPER_LIMIT,
                         FEEDBACK_LOWER_LIMIT, FEEDBACK_UPPER_LIMIT)
except ImportError:
    from safety import (ABSOLUTE_RAW_LOWER_LIMIT, ABSOLUTE_RAW_UPPER_LIMIT,
                        FEEDBACK_LOWER_LIMIT, FEEDBACK_UPPER_LIMIT)


def position_word_to_steps(word):
    if isinstance(word, bool) or not isinstance(word, numbers.Integral) or not 0 <= word <= 65535:
        raise ValueError('position register must be an unsigned 16-bit word')
    return int(word)-65536 if word >= 32768 else int(word)


def validate_feedback_position_steps(position):
    if (isinstance(position, bool) or not isinstance(position, numbers.Real)
            or not math.isfinite(position) or position != int(position)
            or not FEEDBACK_LOWER_LIMIT <= position <= FEEDBACK_UPPER_LIMIT):
        raise ValueError(f'position feedback {position} outside integer tolerance '
                         f'[{FEEDBACK_LOWER_LIMIT}, {FEEDBACK_UPPER_LIMIT}]')
    return int(position)


def validate_position_target_steps(target):
    if (isinstance(target, bool) or not isinstance(target, numbers.Real)
            or not math.isfinite(target) or target != int(target)
            or not ABSOLUTE_RAW_LOWER_LIMIT <= target <= ABSOLUTE_RAW_UPPER_LIMIT):
        raise ValueError(f'position target {target} outside command stroke '
                         f'[{ABSOLUTE_RAW_LOWER_LIMIT}, {ABSOLUTE_RAW_UPPER_LIMIT}]')
    return int(target)


def recovery_exit_target_steps(measured_position):
    """Explicit inward endpoint exit target, NOT a replacement measurement.

    Examples: measured -1 -> target 0; measured 2004 -> target 2000.
    This helper does not issue motion or plan HOME/PARK.
    """
    measured = validate_feedback_position_steps(measured_position)
    return validate_position_target_steps(max(ABSOLUTE_RAW_LOWER_LIMIT,
        min(ABSOLUTE_RAW_UPPER_LIMIT, measured)))
