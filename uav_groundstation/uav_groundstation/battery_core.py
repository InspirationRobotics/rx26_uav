"""battery_core — how long until the low-voltage failsafe, from the voltage trend.

No ROS, no I/O: gcs_node feeds it samples, bench_preflight feeds it a modelled
pack. The page shows its answer in the header on every tab.

WHAT THE AUTOPILOT DOES, which is what this predicts. ArduPilot compares the
LOADED pack voltage against BATT_LOW_VOLT and triggers its failsafe (RTL on
Ekko) when it stays below. Under hover current the pack sags about 0.5 V below
its resting voltage, so on 2026-09-13 the failsafe was projected to arrive with
roughly 30% still in the pack. Time-to-failsafe is therefore a VOLTAGE question.

WHY NOT mAh. The obvious estimate -- capacity minus consumed, divided by current
-- fails twice on Ekko. The autopilot's remaining % assumes the pack was full at
boot (a half-charged pack reads high), and the current sensor still carries the
CubeOrange default calibration, so both current and consumed mAh may be scaled
wrong by an unknown factor.

THE METHOD. Over the last WINDOW_S of armed flight, fit

    V = a - R*I + s*t

by least squares: a is the resting voltage at t=0, R the pack-plus-wiring
resistance, s how fast the RESTING voltage is falling. Then

    loaded voltage now   = a + s*t_now - R*I_recent
    minutes to failsafe  = (loaded now - BATT_LOW_VOLT) / -s / 60

Two properties make this the right fit rather than a clever one:

  * IMMUNE TO THE CURRENT CALIBRATION. If the sensor reads every current k times
    too high, the fit returns R/k, and R*I -- the only place current enters --
    is unchanged. Nothing here trusts the amps' absolute scale.
  * SAG IS SEPARATED FROM DISCHARGE. Fitting V against time alone would read
    every throttle change (a climb, a gust) as the battery emptying. With I in
    the fit, a climb raises I and the model explains the dip as sag.

WHAT IT STILL GETS WRONG, stated so nobody reads the number as a promise: a LiPo's
curve steepens near the end, so a linear trend OVERESTIMATES the time left, more
so the lower the pack. The estimate shortens as that happens, because the window
slides. The autopilot's own failsafe is the real limit; this is a readout.
"""
import math
from collections import deque

#: Seconds of armed flight the trend is fitted over.
WINDOW_S = 180.0
#: No estimate until at least this much armed flight is in the window: a slope
#: over a few seconds is noise.
MIN_SPAN_S = 60.0
#: Below this spread of current (A, one standard deviation), R cannot be fitted
#: and the default below is used instead.
MIN_CURRENT_SPREAD_A = 2.0
#: Pack plus wiring resistance measured from the 2026-09-13 log (16000 mAh
#: pack): 22.66 V at 1.1 A vs 22.14 V at 45.7 A. Only used when the fit cannot
#: separate R itself -- and then the current calibration matters again.
DEFAULT_R_OHM = 0.0115
#: Plausible bounds for a fitted R. Outside them the fit is explaining noise.
R_BOUNDS_OHM = (0.002, 0.08)
#: Current is averaged over this many seconds for the "loaded now" prediction,
#: so one spike does not swing the estimate.
RECENT_S = 20.0
#: Longest estimate worth showing as a number.
MAX_MINUTES = 60.0
#: A break in armed samples longer than this starts the trend again.
GAP_RESET_S = 30.0


def _blank(x):
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else x


def cells_for(low_volt, voltage):
    """Series cell count. From BATT_LOW_VOLT when known (a 6S failsafe sits
    around 3.5-3.6 V/cell, so 21.6 V -> 6), else from the pack voltage."""
    for v, per in ((low_volt, 3.6), (voltage, 3.7)):
        if v is not None and not math.isnan(v) and v > 0:
            return max(1, int(round(v / per)))
    return None


def _solve3(m, y):
    """Solve a 3x3 linear system by Cramer's rule; None if singular."""
    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det(m)
    if abs(d) < 1e-12:
        return None
    out = []
    for col in range(3):
        mc = [row[:] for row in m]
        for r in range(3):
            mc[r][col] = y[r]
        out.append(det(mc) / d)
    return out


def _fit_with_current(samples, t0):
    """Least squares V = a + b*I + s*(t-t0). Returns (a, R, s) or None."""
    n = float(len(samples))
    si = st = sii = stt = sit = sv = siv = stv = 0.0
    for t, v, i in samples:
        tt = t - t0
        si += i
        st += tt
        sii += i * i
        stt += tt * tt
        sit += i * tt
        sv += v
        siv += i * v
        stv += tt * v
    sol = _solve3([[n, si, st], [si, sii, sit], [st, sit, stt]], [sv, siv, stv])
    if sol is None:
        return None
    a, b, s = sol
    return a, -b, s


def _fit_fixed_r(samples, t0, r):
    """Least squares (V + R*I) = a + s*(t-t0) with R given. (a, s) or None."""
    n = float(len(samples))
    xs = [t - t0 for t, _v, _i in samples]
    ys = [v + r * (i if not math.isnan(i) else 0.0) for _t, v, i in samples]
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-9:
        return None
    s = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return my - s * mx, s


class BatteryEstimator:
    """Feed samples as they arrive; ask for a snapshot at the poll rate."""

    def __init__(self):
        self._armed = deque()          # (t, volts, amps) while armed
        self._last = None              # (t, volts, amps, consumed, pct)

    def feed(self, t, volts, amps, consumed_mah, remaining_pct, armed):
        if volts is None or math.isnan(volts):
            return
        self._last = (t, volts, amps, consumed_mah, remaining_pct)
        if armed:
            # A gap means a landing, and possibly a pack swap: a trend fitted
            # across two packs would be nonsense, so each flight starts clean.
            if self._armed and t - self._armed[-1][0] > GAP_RESET_S:
                self._armed.clear()
            self._armed.append((t, volts, amps))
        while self._armed and t - self._armed[0][0] > WINDOW_S:
            self._armed.popleft()

    def clear_trend(self):
        """Forget the armed window, e.g. after a pack swap."""
        self._armed.clear()

    def snapshot(self, now, low_volt, timeout_s):
        """-> dict for the page, or None when no fresh battery reading.

        minutes is None until there is a trend worth extrapolating, and `basis`
        says in words why it is None or what it rests on.
        """
        if self._last is None or now - self._last[0] > timeout_s:
            return None
        t, volts, amps, consumed, pct = self._last
        cells = cells_for(low_volt, volts)
        low_known = low_volt is not None and not math.isnan(low_volt)
        # NaN never leaves this function: the snapshot goes out as JSON, which
        # has no NaN, and a bare NaN token breaks the page's JSON.parse.
        out = {
            "voltage": volts, "current": _blank(amps),
            "consumed_mah": _blank(consumed),
            "remaining_pct": pct, "cells": cells,
            "per_cell": volts / cells if cells else None,
            "low_volt": low_volt if low_known else None,
            "margin_v": volts - low_volt if low_known else None,
            "minutes": None, "r_ohm": None, "basis": "",
        }
        est = self._estimate(now, low_volt if low_known else None)
        out.update(est)
        return out

    def _estimate(self, now, low_volt):
        if low_volt is None:
            return {"basis": "BATT_LOW_VOLT not read from the autopilot yet"}
        s = list(self._armed)
        if len(s) < 10 or s[-1][0] - s[0][0] < MIN_SPAN_S:
            return {"basis": "needs %.0f s of continuous armed flight" % MIN_SPAN_S}
        t0 = s[0][0]
        have_i = all(not math.isnan(i) for _t, _v, i in s)
        r = None
        if have_i:
            mean_i = sum(i for _t, _v, i in s) / len(s)
            spread = math.sqrt(sum((i - mean_i) ** 2 for _t, _v, i in s) / len(s))
            if spread >= MIN_CURRENT_SPREAD_A:
                fit = _fit_with_current(s, t0)
                if fit and R_BOUNDS_OHM[0] <= fit[1] <= R_BOUNDS_OHM[1]:
                    a, r, slope = fit
                    basis = "voltage trend, sag fitted (R %.1f mOhm)" % (r * 1000)
        if r is None:
            r = DEFAULT_R_OHM if have_i else 0.0
            fit = _fit_fixed_r(s, t0, r)
            if fit is None:
                return {"basis": "trend could not be fitted"}
            a, slope = fit
            basis = ("voltage trend, sag from the 13 Sep measurement"
                     if have_i else "voltage trend, no current sensor")
        recent = [i for t, _v, i in s if now - t <= RECENT_S and not math.isnan(i)]
        i_now = sum(recent) / len(recent) if recent else 0.0
        loaded_now = a + slope * (s[-1][0] - t0) - r * i_now
        out = {"r_ohm": r if have_i else None, "basis": basis}
        if slope >= 0:
            out["basis"] = basis + "; not falling yet"
            return out
        minutes = (loaded_now - low_volt) / -slope / 60.0
        out["minutes"] = max(0.0, min(MAX_MINUTES, minutes))
        return out
