"""esc_core — the ESCs' temperatures for the page header: hottest, and trend.

No ROS, no I/O: gcs_node feeds it /uav/esc, gcs_radio.py feeds it the same
MAVLink blocks straight off the radio, bench_preflight feeds it numbers.

WHY IT IS ON THE PAGE. On 2026-10-13, after the cover and battery mount took
Ekko to 5.4 kg, a 4.5 min hover drove the 4-in-1 ESC from 32 C to 96 C and it
was still climbing about 2 C every 20 s -- while the 22 Sep flights at 4.4 kg
never passed 50 C. AM32 can be set to cut power at a temperature, and over water
a cut motor is the aircraft. The battery was never the limit that day; this was.

THE TREND. The hottest temperature is fitted against time over the last TREND_S,
by least squares, because the ESC reports whole degrees and a two-point
difference over a few seconds would swing on one degree of rounding.

BLANKS OVER GUESSES. Each ESC is kept with its own receipt time; one that goes
quiet drops out, and when none is fresh the snapshot is None -- never the last
temperature heard, which would look like a reading.
"""
from collections import deque

#: Amber from here: hot enough to watch.
WARN_C = 80.0
#: Red from here. 2026-10-13's hover peaked at 96 C. Set it ~10 C below the
#: ESC's AM32 temperature limit once that setting has been read.
BAD_C = 95.0
#: Seconds the trend is fitted over.
TREND_S = 60.0
#: No trend until the window spans this much: a slope over a few samples of
#: whole degrees is rounding, not heating.
MIN_TREND_S = 15.0
#: Below this rate either way, the page says "steady".
STEADY_C_PER_MIN = 1.0


class EscTracker:
    def __init__(self, timeout_s):
        self.timeout_s = float(timeout_s)
        self._esc = {}               # output index -> (temp_c, rpm, recv_t)
        self._base = None            # lowest output index ever heard
        self._hist = deque()         # (t, hottest_c)

    def feed(self, now, entries):
        """entries: [(index, temperature_c, rpm), ...] -- one block, or all."""
        for index, temp_c, rpm in entries:
            self._esc[int(index)] = (float(temp_c), float(rpm), now)
            self._base = int(index) if self._base is None else min(self._base, int(index))
        fresh = self._fresh(now)
        if not fresh:
            return
        if self._hist and now - self._hist[-1][0] > self.timeout_s:
            self._hist.clear()       # a gap: a fit across it would be a guess
        self._hist.append((now, max(v[0] for v in fresh.values())))
        while self._hist and now - self._hist[0][0] > TREND_S:
            self._hist.popleft()

    def _fresh(self, now):
        return {i: v for i, v in self._esc.items()
                if now - v[2] <= self.timeout_s}

    def _rate_c_per_min(self):
        if len(self._hist) < 3 or self._hist[-1][0] - self._hist[0][0] < MIN_TREND_S:
            return None
        t0 = self._hist[0][0]
        xs = [t - t0 for t, _ in self._hist]
        ys = [c for _, c in self._hist]
        mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx <= 0:
            return None
        return 60.0 * sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx

    def snapshot(self, now):
        """-> dict for the page, or None when no ESC has reported recently."""
        fresh = self._fresh(now)
        if not fresh:
            return None
        # ESC n counts from the lowest output EVER heard, so a 4-in-1 on
        # outputs 9-12 reads ESC 1-4 and one going quiet does not renumber the
        # rest. The output itself is in the tooltip.
        temps = [{"esc": i - self._base + 1, "output": i + 1, "c": fresh[i][0],
                  "rpm": fresh[i][1]} for i in sorted(fresh)]
        hottest = max(temps, key=lambda e: e["c"])
        state = ("bad" if hottest["c"] >= BAD_C else
                 "warn" if hottest["c"] >= WARN_C else "ok")
        return {"temps": temps, "max_c": hottest["c"], "hottest": hottest["esc"],
                "rate_c_min": self._rate_c_per_min(), "state": state,
                "warn_c": WARN_C, "bad_c": BAD_C,
                "steady_c_min": STEADY_C_PER_MIN,
                "age_s": now - max(v[2] for v in fresh.values())}
