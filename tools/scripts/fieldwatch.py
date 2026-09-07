#!/usr/bin/env python3
"""Live anomaly watcher for a flying session. Run on the JETSON HOST.

    python3 ~/robotx_ws/src/rx26_uav/tools/scripts/fieldwatch.py

WHY IT BINDS 14542 AND NOTHING ELSE
-----------------------------------
MAVProxy on the host owns /dev/uav-pixhawk and rebroadcasts. Port 14541 belongs
to telemetry_bridge -- the flight stack. A `udpin` bind STEALS datagrams: the
displaced consumer sees silence, not an error. That has already cost one
sortie, where 1,025 frames were written with a blank pose because this script's
ancestor bound 14541. 14542 is the ad-hoc tooling port. Do not change it.

WHAT IT IS FOR
--------------
An operator at the field is holding a controller and cannot read a firehose.
So this is EDGE TRIGGERED: it prints when a condition changes, not every time
it is true, and a condition must persist for DWELL_S before it counts. The
normal output for a healthy flight is almost nothing.

Samples go to /tmp/fieldwatch.jsonl at SAMPLE_HZ for post-flight review. That
file is a convenience, not a record -- the frame index is the record.
"""

import json
import os
import sys
import time
from collections import deque

from pymavlink import mavutil

PORT = 14542                 # NEVER 14541. See the note above.
DWELL_S = 2.0                # a condition must hold this long to be reported
SAMPLE_HZ = 5.0
SAMPLE_PATH = "/tmp/fieldwatch.jsonl"
GAP_S = 3.0                  # no telemetry for this long is itself an event

# Thresholds. Deliberately conservative: a false alarm costs a glance, a missed
# one costs an airframe.
VIBE_WARN, VIBE_CRIT = 30.0, 60.0
EKF_VAR = 0.8
GPS_MIN_SATS = 8
ESC_TEMP_C = 80
ESC_RPM_SPREAD = 0.12        # fraction from the median of the four
ESC_RPM_FLOOR = 1500         # below this the props are not really turning


def stamp():
    return time.strftime("%H:%M:%S")


def emit(level, text):
    print("%s  %-5s %s" % (stamp(), level, text), flush=True)


class Alarm:
    """One edge-triggered condition.

    Holds a condition for DWELL_S before firing, and fires again only when it
    clears and returns. This is the whole reason the output stays readable --
    vibration hovering either side of a threshold would otherwise print
    continuously and train the operator to ignore it.
    """

    def __init__(self, name):
        self.name = name
        self.since = None
        self.firing = False

    def update(self, bad, now, msg, level="WARN"):
        if bad:
            if self.since is None:
                self.since = now
            elif not self.firing and now - self.since >= DWELL_S:
                self.firing = True
                emit(level, msg)
        else:
            if self.firing:
                emit("OK", "%s cleared" % self.name)
            self.since = None
            self.firing = False


def main():
    alarms = {}

    def alarm(key):
        if key not in alarms:
            alarms[key] = Alarm(key)
        return alarms[key]

    emit("INFO", "binding udpin:127.0.0.1:%d (tooling port)" % PORT)
    try:
        m = mavutil.mavlink_connection("udpin:127.0.0.1:%d" % PORT)
    except Exception as e:
        emit("FATAL", "cannot bind %d: %s" % (PORT, e))
        return 1

    if not m.wait_heartbeat(timeout=15):
        emit("FATAL", "no heartbeat on %d in 15 s -- is MAVProxy running "
                      "and does start_mavproxy.sh have --out=udp:127.0.0.1:%d?"
             % (PORT, PORT))
        return 1
    emit("INFO", "heartbeat from system %d -- watching" % m.target_system)

    state = {}
    armed = None
    clip_last = None
    last_rx = time.time()
    last_sample = 0.0
    arm_t = None
    peak_alt = 0.0
    peak_vibe = 0.0

    try:
        sample_f = open(SAMPLE_PATH, "a", buffering=1)
    except OSError:
        sample_f = None
        emit("WARN", "cannot write %s -- continuing without samples" % SAMPLE_PATH)

    while True:
        msg = m.recv_match(blocking=True, timeout=1.0)
        now = time.time()

        # ---- link gap ------------------------------------------------------
        alarm("telemetry").update(
            now - last_rx > GAP_S, now,
            "no telemetry for %.0f s" % (now - last_rx), "CRIT")
        if msg is None:
            continue
        last_rx = now
        t = msg.get_type()

        if t == "HEARTBEAT":
            a = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed is None:
                armed = a
                emit("INFO", "armed" if a else "disarmed")
            elif a != armed:
                armed = a
                if a:
                    arm_t = now
                    peak_alt = peak_vibe = 0.0
                    emit("INFO", "ARMED")
                else:
                    dur = (now - arm_t) if arm_t else 0.0
                    emit("INFO", "DISARMED  flight %.0f s  peak alt %.1f m  "
                                 "peak vibe %.0f" % (dur, peak_alt, peak_vibe))

        elif t == "VIBRATION":
            v = max(msg.vibration_x, msg.vibration_y, msg.vibration_z)
            state["vibe"] = v
            peak_vibe = max(peak_vibe, v)
            alarm("vibration").update(
                v >= VIBE_CRIT, now, "VIBRATION %.0f (crit >%.0f) -- land" % (v, VIBE_CRIT), "CRIT")
            if v < VIBE_CRIT:
                alarm("vibe_warn").update(
                    v >= VIBE_WARN, now, "vibration %.0f (warn >%.0f)" % (v, VIBE_WARN))
            clip = (msg.clipping_0, msg.clipping_1, msg.clipping_2)
            if clip_last is not None and any(c > p for c, p in zip(clip, clip_last)):
                emit("CRIT", "accel CLIPPING %s -- vibration is saturating the IMU" % (clip,))
            clip_last = clip

        elif t == "EKF_STATUS_REPORT":
            worst = max(msg.velocity_variance, msg.pos_horiz_variance,
                        msg.compass_variance)
            state["ekf"] = worst
            alarm("ekf").update(worst > EKF_VAR, now,
                                "EKF variance %.2f (>%.1f)" % (worst, EKF_VAR))

        elif t == "GPS_RAW_INT":
            state["sats"] = msg.satellites_visible
            state["fix"] = msg.fix_type
            alarm("gps").update(
                msg.fix_type < 3 or msg.satellites_visible < GPS_MIN_SATS, now,
                "GPS degraded: fix %d, %d sats" % (msg.fix_type, msg.satellites_visible))

        elif t == "GLOBAL_POSITION_INT":
            alt = msg.relative_alt / 1000.0
            state["alt"] = alt
            peak_alt = max(peak_alt, alt)

        elif t.startswith("ESC_TELEMETRY"):
            rpm = [r for r in getattr(msg, "rpm", []) if r]
            temp = list(getattr(msg, "temperature", []))
            if temp:
                hot = max(temp)
                state["esc_temp"] = hot
                alarm("esc_temp").update(hot >= ESC_TEMP_C, now,
                                         "ESC temp %d C (>%d)" % (hot, ESC_TEMP_C), "CRIT")
            if len(rpm) >= 4 and max(rpm) > ESC_RPM_FLOOR:
                med = sorted(rpm)[len(rpm) // 2]
                if med:
                    spread = max(abs(r - med) / float(med) for r in rpm)
                    state["esc_spread"] = spread
                    # One motor working much harder than its peers is the
                    # signature of a damaged prop or a failing bearing.
                    alarm("esc_spread").update(
                        spread > ESC_RPM_SPREAD, now,
                        "ESC RPM outlier: %s (%.0f%% from median)"
                        % (rpm, spread * 100), "CRIT")

        elif t == "SYS_STATUS":
            v = msg.voltage_battery / 1000.0
            state["volts"] = v

        elif t == "STATUSTEXT":
            txt = msg.text.strip() if isinstance(msg.text, str) else \
                msg.text.decode("utf-8", "replace").strip()
            if txt:
                emit("FC", txt)

        # ---- sampling ------------------------------------------------------
        if sample_f and now - last_sample >= 1.0 / SAMPLE_HZ:
            last_sample = now
            rec = dict(state)
            rec["t"] = round(now, 2)
            rec["armed"] = armed
            try:
                sample_f.write(json.dumps(rec) + "\n")
            except OSError:
                pass


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        emit("INFO", "stopped")
