"""Motor g <-> nominal SI force; no UART access and no sensor Tare.

The manufacturer calls the native signed reading g. This nominal mass-equivalent
conversion is not a sensor calibration. Per-axis calibration and direction remain
explicit at the conversion boundary. Scalars and ID1..ID6 vectors are supported.
"""
import math
import numbers

STANDARD_GRAVITY_M_S2 = 9.80665
NOMINAL_N_PER_G = STANDARD_GRAVITY_M_S2 / 1000.0
NOMINAL_G_PER_N = 1.0 / NOMINAL_N_PER_G
INT16_MIN = -32768
INT16_MAX = 32767
# Preserve the existing absolute protection range, formerly +/-1500 g.
DEFAULT_FORCE_LIMIT_N = 1500.0 * NOMINAL_N_PER_G


def _six(value, name):
    values = [float(x) for x in value]
    if len(values) != 6 or not all(math.isfinite(x) for x in values):
        raise ValueError(f'{name} must contain six finite values')
    return values


def _convert(value, direction, scale_n_per_g, inverse):
    scalar = isinstance(value, numbers.Real)
    values = [float(value)] if scalar else _six(value, 'force')
    directions = ([float(direction)] * len(values) if isinstance(direction, numbers.Real)
                  else _six(direction, 'direction'))
    scales = ([float(scale_n_per_g)] * len(values) if isinstance(scale_n_per_g, numbers.Real)
              else _six(scale_n_per_g, 'scale_n_per_g'))
    if len(directions) != len(values) or len(scales) != len(values):
        raise ValueError('scalar force requires scalar calibration')
    if not all(math.isfinite(x) for x in values):
        raise ValueError('force must be finite')
    if any(d not in (-1.0, 1.0) for d in directions):
        raise ValueError('direction must be +1 or -1')
    if any(not math.isfinite(s) or s <= 0 for s in scales):
        raise ValueError('scale_n_per_g must be positive and finite')
    result = [v / (d*s) if inverse else v*d*s for v, d, s in zip(values, directions, scales)]
    if not all(math.isfinite(x) for x in result):
        raise ValueError('converted force must be finite')
    return result[0] if scalar else result


def g_to_n(force_g_raw, direction=1.0, scale_n_per_g=NOMINAL_N_PER_G):
    """Signed native g -> N, applying calibration and axis direction once."""
    return _convert(force_g_raw, direction, scale_n_per_g, False)


def n_to_g(target_force_n, direction=1.0, scale_n_per_g=NOMINAL_N_PER_G):
    """N -> signed native g (float); quantize only at the register boundary."""
    return _convert(target_force_n, direction, scale_n_per_g, True)


def quantize_force_g(target_force_g):
    """Nearest integer g, ties to even (existing Python round semantics).

    Reject overflow instead of wrapping/silently clipping a force command.
    Fractional inputs inside the int16 interval may round to either endpoint.
    """
    def one(value):
        value = float(value)
        # Permit only machine-roundoff at endpoints after an SI round trip.
        tolerance = 4 * math.ulp(float(INT16_MIN))
        if not math.isfinite(value) or not INT16_MIN-tolerance <= value <= INT16_MAX+tolerance:
            raise ValueError('force outside signed 16-bit range [-32768, 32767] g')
        return int(round(max(INT16_MIN, min(INT16_MAX, value))))
    return one(target_force_g) if isinstance(target_force_g, numbers.Real) else [one(v) for v in _six(target_force_g, 'target_force_g')]


def encode_signed_i16_to_u16(value):
    if not isinstance(value, numbers.Integral) or isinstance(value, bool) or not INT16_MIN <= value <= INT16_MAX:
        raise ValueError('Value outside signed 16-bit range [-32768, 32767]')
    return int(value) & 0xffff


def decode_u16_to_signed_i16(value):
    if not isinstance(value, numbers.Integral) or isinstance(value, bool) or not 0 <= value <= 65535:
        raise ValueError('Value outside unsigned 16-bit range [0, 65535]')
    return int(value) - 65536 if value >= 32768 else int(value)


def clamp_force_n(target_force_n, min_force_n, max_force_n):
    """Per-axis SI saturation; scalar bounds may be broadcast to six axes."""
    force_n = _six(target_force_n, 'target_force_n')
    lower = [float(min_force_n)]*6 if isinstance(min_force_n, numbers.Real) else _six(min_force_n, 'min_force_n')
    upper = [float(max_force_n)]*6 if isinstance(max_force_n, numbers.Real) else _six(max_force_n, 'max_force_n')
    if any(not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi for lo, hi in zip(lower, upper)):
        raise ValueError('force bounds must be finite and min_force_n < max_force_n')
    return [max(lo, min(hi, f)) for f, lo, hi in zip(force_n, lower, upper)]


def quantize_force_g_with_n_limits(target_force_g, min_force_n, max_force_n,
                                   direction=1.0, scale_n_per_g=NOMINAL_N_PER_G):
    """Encode integer g without rounding past an SI force cap.

    The input is already saturated/slew-limited. Only bound endpoints are
    converted here; the target is not converted a second time.
    """
    clamp_force_n([0.0]*6, min_force_n, max_force_n)  # validate SI bounds
    lower_n = [float(min_force_n)]*6 if isinstance(min_force_n, numbers.Real) else min_force_n
    upper_n = [float(max_force_n)]*6 if isinstance(max_force_n, numbers.Real) else max_force_n
    a = n_to_g(lower_n, direction, scale_n_per_g)
    b = n_to_g(upper_n, direction, scale_n_per_g)
    def snap_roundoff(v):
        rounded = round(v)
        return float(rounded) if abs(v-rounded) <= 4*math.ulp(v) else v
    encoded = quantize_force_g(target_force_g)
    result = []
    for value, x, y in zip(encoded, a, b):
        lower = max(INT16_MIN, math.ceil(snap_roundoff(min(x,y))))
        upper = min(INT16_MAX, math.floor(snap_roundoff(max(x,y))))
        if lower > upper:
            raise ValueError('no representable integer g within SI force limits')
        result.append(max(lower, min(upper, value)))
    return result
