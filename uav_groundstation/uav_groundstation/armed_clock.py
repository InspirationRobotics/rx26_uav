"""armed_clock — how long the aircraft has been armed since it was powered on.

For Chris's flight log: armed is taken as flying, and the page shows the total
for this power-on in the header, so it can be written down at the end of a
session without adding up sorties by hand.

WHAT "THIS POWER-ON" MEANS. The Jetson and the autopilot are powered from the
same battery, so a battery swap reboots both. The Linux boot id
(/proc/sys/kernel/random/boot_id, the same inside the container as on the host)
changes on every boot, and the total is saved to a small file tagged with it:
a ground_station restart mid-session picks the total back up, a power cycle
starts from zero.

BLANKS OVER GUESSES. Time only counts while a FRESH FcuStatus says armed. A
telemetry dropout neither adds time nobody saw nor ends the session, and a
flight is counted only on a disarmed -> armed edge that was actually observed,
so a blip mid-flight does not turn one flight into two.

A FLICKER IS NOT A DROPOUT. The autopilot's heartbeat arrives once a second and
the staleness timeout is also a second, so the armed state blinks "unknown" for
a moment every few seconds (seen in the ROS test, 2026-09-14). Closing the count
on every blink would quietly shave flight time. An unknown shorter than
UNKNOWN_GRACE_S keeps counting; a longer one stops the count at the moment the
state went unknown.

No ROS here; gcs_node feeds it and bench_preflight drives it.
"""
import json
import os

#: How often the running total is written while armed, so a crash of the ground
#: station loses at most this much of the count.
SAVE_EVERY_S = 10.0
#: An armed state unknown for less than this is a flicker, not a dropout.
UNKNOWN_GRACE_S = 3.0
BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"


def read_boot_id(path=BOOT_ID_PATH):
    try:
        with open(path) as f:
            return f.read().strip() or None
    except OSError:
        return None


class ArmedClock:

    def __init__(self, path, boot_id):
        self.path = path
        self.boot_id = boot_id
        self.total_s = 0.0
        self.flights = 0
        self._since = None          # monotonic start of the current armed stretch
        self._last_known = None     # last armed state we actually saw
        self._saved_t = None
        self._unknown_since = None  # when the armed state last went unknown
        self.resumed = False
        self._load()

    def _load(self):
        if not (self.path and self.boot_id):
            return
        try:
            with open(self.path) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return
        if d.get("boot_id") == self.boot_id:
            self.total_s = float(d.get("armed_s", 0.0))
            self.flights = int(d.get("flights", 0))
            self.resumed = True

    def _save(self, now):
        self._saved_t = now
        if not (self.path and self.boot_id):
            return
        tmp = self.path + ".part"
        try:
            with open(tmp, "w") as f:
                json.dump({"boot_id": self.boot_id,
                           "armed_s": round(self.seconds(now), 1),
                           "flights": self.flights}, f)
            os.replace(tmp, self.path)
        except OSError:
            pass                   # a readout, not a flight record; never fatal

    def update(self, now, known, armed):
        """Call at any rate. known=False means the armed state is not fresh."""
        if not known:
            if self._unknown_since is None:
                self._unknown_since = now
            if (self._since is not None
                    and now - self._unknown_since >= UNKNOWN_GRACE_S):
                # A real dropout: count up to when it went unknown, no further.
                self.total_s += self._unknown_since - self._since
                self._since = None
                self._save(now)
            return
        self._unknown_since = None
        if armed:
            if self._since is None:
                self._since = now
                if self._last_known is False:
                    self.flights += 1
            if self._saved_t is None or now - self._saved_t >= SAVE_EVERY_S:
                self._save(now)
        elif self._since is not None:
            self.total_s += now - self._since
            self._since = None
            self._save(now)
        self._last_known = bool(armed)

    def seconds(self, now):
        if self._since is None:
            return self.total_s
        end = now
        if self._unknown_since is not None and now - self._unknown_since >= UNKNOWN_GRACE_S:
            end = self._unknown_since
        return self.total_s + (end - self._since)

    def snapshot(self, now):
        return {"seconds": round(self.seconds(now), 1), "flights": self.flights,
                "armed": self._since is not None, "resumed": self.resumed}
