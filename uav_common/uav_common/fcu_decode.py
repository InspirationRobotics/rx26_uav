"""fcu_decode — MAVLink wire values -> the units and blanks our messages carry.

PURE. telemetry_bridge calls these on the RX thread; bench_preflight calls them
with hand-written numbers. Every MAVLink "unknown" sentinel becomes NaN (or the
message's own unknown value) here, in one place, so no consumer ever mistakes
a -1 or a 65535 for a measurement.
"""
import math

NAN = float("nan")

#: Autopilot parameters the ground station reads back (see FcuParams.msg), keyed
#: by MAVLink name -> FcuParams field.
FCU_PARAMS = {
    "BATT_LOW_VOLT": "batt_low_volt",
    "BATT_CRT_VOLT": "batt_crt_volt",
    "BATT_CAPACITY": "batt_capacity_mah",
    "FENCE_ENABLE": "fence_enable",
    "FENCE_ALT_MAX": "fence_alt_max",
    "FENCE_TYPE": "fence_type",
    "FENCE_MARGIN": "fence_margin",
    "FENCE_ACTION": "fence_action",
}

#: FENCE_TYPE bits.
FENCE_TYPE_ALT_MAX = 1
FENCE_TYPE_CIRCLE = 2
FENCE_TYPE_POLYGON = 4


def battery_from_sys_status(voltage_battery_mv, current_battery_ca,
                            battery_remaining):
    """SYS_STATUS battery fields -> (volts, amps, remaining_pct).

    current_battery is centi-amps with -1 for "no sensor"; battery_remaining is
    percent with -1 for "no estimate"; voltage_battery is millivolts with
    UINT16_MAX for "unknown".
    """
    volts = NAN if voltage_battery_mv in (None, 65535) else voltage_battery_mv / 1000.0
    amps = NAN if current_battery_ca in (None, -1) else current_battery_ca / 100.0
    pct = -1 if battery_remaining in (None, -1) else int(battery_remaining)
    return volts, amps, pct


def consumed_mah(current_consumed):
    """BATTERY_STATUS.current_consumed (mAh, -1 unknown) -> mAh or NaN."""
    return NAN if current_consumed in (None, -1) else float(current_consumed)


def gps_from_raw(fix_type, eph, satellites_visible, h_acc_mm):
    """GPS_RAW_INT -> (fix_type, satellites, hdop, h_acc_m).

    eph is HDOP x100 with UINT16_MAX unknown; satellites_visible is 255 unknown;
    h_acc is millimetres with 0 meaning "not reported" (a MAVLink 2 extension,
    absent entirely on a MAVLink 1 link).
    """
    hdop = NAN if eph in (None, 65535) else eph / 100.0
    sats = 255 if satellites_visible is None else int(satellites_visible)
    acc = NAN if not h_acc_mm else h_acc_mm / 1000.0
    return int(fix_type or 0), sats, hdop, acc


def param_name(raw):
    """PARAM_VALUE.param_id -> clean str. Arrives NUL-padded, as str or bytes."""
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", "replace")
    return (raw or "").split("\x00", 1)[0].strip()


def is_nan(x):
    return isinstance(x, float) and math.isnan(x)
