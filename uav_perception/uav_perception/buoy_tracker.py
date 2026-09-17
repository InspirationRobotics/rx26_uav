"""buoy_tracker — sightings in, one buoy per physical buoy out, each with a state.

PURE. No ROS, no clock: every call carries its own time `t` (seconds, the frame's
arrival time), so the live mapper and the replay tool reach the same answer from
the same sightings, and a bench can drive a 4-second flash in microseconds.

TWO JOBS, deliberately kept separate:

  WHERE. A sighting within assoc_radius_m of a known buoy is that buoy; anything
  farther starts a new one. Identity comes from the ground position, never from
  where a box sits in the frame -- left/right in the image swaps every time the
  aircraft turns. The position is the weighted mean of every sighting (overhead
  ones count more, see geolocate.centre_weight), and the weighted RMS spread is
  reported beside it as the map's self-check.

  "Within assoc_radius_m" is measured to the buoy's MEAN *or* to its most recent
  sightings, whichever is nearer. Non-RTK GPS wanders slowly -- half a metre to a
  metre over tens of seconds -- and every sighting wanders with it. Measured to
  the mean alone, a wander past assoc_radius_m started a SECOND buoy on top of
  the first (simulated at 0.6 m of wander: one buoy reported as two to four).
  Recent sightings move with the wander, so the track follows it. Two genuinely
  separate buoys 3 m apart still never share a track, because whichever track's
  sightings are nearer wins.

  Tracks that end up overlapping are merged: means closer than merge_radius_m,
  or closer than 1.5x their combined spread -- but NEVER farther apart than
  assoc_radius_m, so the 3 m gate buoys cannot be merged however noisy it gets.

  WHAT. The Task 1 beacon flashes 1 s on / 1 s off, so a single frame cannot tell
  FLASHING from SOLID, or a flashing buoy in its dark second from an OFF one. The
  state is decided from how often the buoy was seen LIT across at least
  min_observe_s of watching:

      lit share >= solid_min_lit                          SOLID  <colour>
      lit share <= off_max_lit                            OFF
      in between, with >= min_flash_transitions changes   FLASHING <colour>
      in between, without alternation                     UNKNOWN (keep watching:
                                                          that pattern is a buoy
                                                          CHANGING, not flashing)

  Watch time only accumulates between samples no more than max_sample_gap_s
  apart, so flying away and coming back adds the time actually spent looking,
  not the time spent elsewhere.

WHAT THE CALLER MUST ALREADY HAVE FILTERED: only full-view detections come in.
A buoy clipped by the frame edge usually has its beacon out of frame (so it reads
"dark") and its box centre is the centre of the visible part only. Both would
poison exactly the two things this file computes.
"""
import math
from collections import deque

from uav_common import geo

#: Model class name -> beacon colour. Anything else that is not UNLIT is ignored
#: for state (the sighting still counts toward position).
LIT_COLOURS = {"red": "RED", "green": "GREEN", "blue": "BLUE"}
UNLIT = {"dark", "off"}

STATE_UNKNOWN = "UNKNOWN"
STATE_OFF = "OFF"
STATE_FLASHING = "FLASHING"
STATE_SOLID = "SOLID"

# Bounds so a long hover cannot grow memory without limit. At 4 Hz inference,
# 2400 samples is ten minutes over one buoy -- far past the four seconds a state
# needs -- and positions beyond 600 sightings no longer move the mean.
MAX_SAMPLES = 2400
MAX_SIGHTINGS = 600

# Sightings a track "remembers" for association: ~10 s at 4 Hz, the timescale
# GPS wander builds up over. Longer lets one outlier widen the net for too long.
RECENT_SIGHTINGS = 40

# Tracks whose means are closer than this many times their combined spread are
# one buoy seen through wandering GPS. Always capped at assoc_radius_m.
OVERLAP_FACTOR = 1.5


class TrackerConfig:
    """Decision thresholds. All read-only on the aircraft; see uav_params.yaml."""

    def __init__(self, *, assoc_radius_m=1.5, merge_radius_m=1.0,
                 min_sightings=3, min_observe_s=4.0, min_samples=12,
                 max_sample_gap_s=1.0, solid_min_lit=0.85, off_max_lit=0.15,
                 min_flash_transitions=2, min_colour_agreement=0.6,
                 lock_state=True, decide_window_s=0.0):
        if not 0.0 <= off_max_lit < solid_min_lit <= 1.0:
            raise ValueError("need 0 <= off_max_lit < solid_min_lit <= 1, got "
                             "%r, %r" % (off_max_lit, solid_min_lit))
        if merge_radius_m > assoc_radius_m:
            raise ValueError("merge_radius_m must not exceed assoc_radius_m")
        self.decide_window_s = float(decide_window_s)
        self.assoc_radius_m = float(assoc_radius_m)
        self.merge_radius_m = float(merge_radius_m)
        self.min_sightings = int(min_sightings)
        self.min_observe_s = float(min_observe_s)
        self.min_samples = int(min_samples)
        self.max_sample_gap_s = float(max_sample_gap_s)
        self.solid_min_lit = float(solid_min_lit)
        self.off_max_lit = float(off_max_lit)
        self.min_flash_transitions = int(min_flash_transitions)
        self.min_colour_agreement = float(min_colour_agreement)
        self.lock_state = bool(lock_state)


class Track:
    """One physical buoy: its sightings and its colour samples."""

    def __init__(self, track_id):
        self.id = track_id
        self.sightings = deque(maxlen=MAX_SIGHTINGS)   # (x, y, weight)
        self.recent = deque(maxlen=RECENT_SIGHTINGS)   # (x, y), for association
        self.samples = deque(maxlen=MAX_SAMPLES)       # (t, lit, colour or None)
        self.last_t = None
        self.locked = None                             # (state, colour) once frozen

    # ---- where

    def distance_to(self, x, y):
        """Nearer of the mean and the most recent sightings. See module docstring."""
        mx, my = self.mean_xy()
        d = math.hypot(x - mx, y - my)
        for rx, ry in self.recent:
            d = min(d, math.hypot(x - rx, y - ry))
        return d

    def mean_xy(self):
        wsum = sum(w for _, _, w in self.sightings)
        return (sum(x * w for x, _, w in self.sightings) / wsum,
                sum(y * w for _, y, w in self.sightings) / wsum)

    def spread_m(self):
        mx, my = self.mean_xy()
        wsum = sum(w for _, _, w in self.sightings)
        return math.sqrt(sum(w * ((x - mx) ** 2 + (y - my) ** 2)
                             for x, y, w in self.sightings) / wsum)

    # ---- what

    def add(self, t, x, y, weight, class_name):
        self.sightings.append((x, y, weight))
        self.recent.append((x, y))
        name = (class_name or "").lower()
        if name in LIT_COLOURS:
            self.samples.append((t, True, LIT_COLOURS[name]))
        elif name in UNLIT:
            self.samples.append((t, False, None))
        self.last_t = t if self.last_t is None else max(self.last_t, t)

    def absorb(self, other):
        """Merge another track that turned out to be this buoy."""
        self.sightings.extend(other.sightings)
        self.recent.extend(other.recent)
        merged = sorted(list(self.samples) + list(other.samples),
                        key=lambda s: s[0])
        self.samples = deque(merged, maxlen=MAX_SAMPLES)
        if other.last_t is not None:
            self.last_t = (other.last_t if self.last_t is None
                           else max(self.last_t, other.last_t))
        if self.locked is None:
            self.locked = other.locked

    def evidence(self, cfg):
        """The numbers a state is decided from.

        With cfg.decide_window_s > 0 only the samples from the last window count.
        THAT IS WHAT MAKES A CHANGE VISIBLE: weighing every sample ever taken, a
        buoy watched as red for a minute needs another minute of dark samples
        before the average moves, and Task 1's Disruptive tier changes the
        passage under a boat that is already driving it. Positions still use
        every sighting -- only the STATE decision is windowed.
        """
        samples = sorted(self.samples, key=lambda s: s[0])
        if cfg.decide_window_s > 0 and samples:
            cut = samples[-1][0] - cfg.decide_window_s
            samples = [s for s in samples if s[0] >= cut]
        n = len(samples)
        lit = [s for s in samples if s[1]]
        observed = 0.0
        transitions = 0
        for prev, cur in zip(samples, samples[1:]):
            dt = cur[0] - prev[0]
            if 0.0 < dt <= cfg.max_sample_gap_s:
                observed += dt
            if cur[1] != prev[1]:
                transitions += 1
        colour, agreement = "", 0.0
        if lit:
            counts = {}
            for s in lit:
                counts[s[2]] = counts.get(s[2], 0) + 1
            colour = max(counts, key=counts.get)
            agreement = counts[colour] / float(len(lit))
        return {"samples": n, "lit_fraction": (len(lit) / float(n)) if n else 0.0,
                "flash_transitions": transitions, "observed_s": observed,
                "colour": colour, "colour_agreement": agreement}

    def decide(self, cfg, ev):
        """-> (state, colour) from evidence. See the module docstring's table."""
        need = cfg.min_observe_s
        if cfg.decide_window_s > 0:
            # The watch time can never fill the whole window (it is the sum of
            # the gaps between samples inside it), so asking for the full
            # min_observe_s within a window of the same length decides nothing,
            # ever.
            need = min(need, 0.8 * cfg.decide_window_s)
        if ev["samples"] < cfg.min_samples or ev["observed_s"] < need:
            return STATE_UNKNOWN, ""
        frac = ev["lit_fraction"]
        if frac <= cfg.off_max_lit:
            return STATE_OFF, ""
        if ev["colour_agreement"] < cfg.min_colour_agreement:
            return STATE_UNKNOWN, ""
        if frac >= cfg.solid_min_lit:
            return STATE_SOLID, ev["colour"]
        if ev["flash_transitions"] >= cfg.min_flash_transitions:
            return STATE_FLASHING, ev["colour"]
        return STATE_UNKNOWN, ""


def label(state, colour):
    """'FLASHING_BLUE', 'SOLID_BLUE', 'OFF', 'UNKNOWN'."""
    return "%s_%s" % (state, colour) if colour else state


class BuoyTracker:
    """All the buoys seen so far, in one local metric frame."""

    def __init__(self, cfg=None):
        self.cfg = cfg or TrackerConfig()
        self.clear()

    def unlock_all(self):
        """Let every decided state be re-decided.

        Called when the tier changes to Disruptive: a state frozen under
        Advanced would otherwise never move again, and the whole tier is about
        states that move.
        """
        for tr in self.tracks:
            tr.locked = None

    def clear(self):
        self.origin = None          # (lat, lon) of the first sighting
        self.tracks = []
        self._next_id = 1
        self._merged_into = {}      # retired id -> surviving id

    def add(self, t, lat, lon, weight, class_name):
        """One full-view sighting. Returns the id of the buoy it was assigned to."""
        return self.add_frame(t, [(lat, lon, weight, class_name)])[0]

    def add_frame(self, t, sightings):
        """Every full-view sighting from ONE frame -> their buoy ids, in order.

        Assigned together, because two boxes in the same frame can never be the
        same buoy -- and because GPS wander moves every box in a frame by the
        same amount. Pairing the closest (track, sighting) first, each used once,
        lets the relative layout of the buoys in the frame decide who is who, and
        that layout does not wander. Assigning one box at a time instead let two
        buoys 3 m apart swap samples at 1 m of simulated wander, and both then
        read as flashing.
        """
        if not sightings:
            return []
        if self.origin is None:
            self.origin = (sightings[0][0], sightings[0][1])
        xy = [geo.latlon_to_xy(lat, lon, self.origin) for lat, lon, _, _ in sightings]
        pairs = sorted((tr.distance_to(x, y), i, k)
                       for i, (x, y) in enumerate(xy)
                       for k, tr in enumerate(self.tracks))
        owner = [None] * len(sightings)
        taken = set()
        for d, i, k in pairs:
            if d > self.cfg.assoc_radius_m:
                break
            if owner[i] is None and k not in taken:
                owner[i] = self.tracks[k]
                taken.add(k)
        ids = []
        for i, (lat, lon, weight, class_name) in enumerate(sightings):
            tr = owner[i]
            if tr is None:
                tr = Track(self._next_id)
                self._next_id += 1
                self.tracks.append(tr)
            tr.add(t, xy[i][0], xy[i][1], weight, class_name)
            ids.append(tr.id)
        self._merge_close()
        # A merge may have retired a track assigned above; report its survivor.
        alive = {tr.id for tr in self.tracks}
        return [i if i in alive else self._survivor(i) for i in ids]

    def _survivor(self, gone_id):
        """The track a merged-away id now lives in (the lowest id that absorbed it)."""
        return self._merged_into.get(gone_id, gone_id)

    def _merge_close(self):
        """Two tracks that drift within merge_radius_m are one buoy seen twice.

        Happens when the first sightings of a buoy come from the frame edge,
        land more than assoc_radius_m apart, and start two tracks that later
        overhead sightings pull together. The older id survives, so a buoy
        already reported keeps its number.
        """
        merged = True
        while merged:
            merged = False
            for i, a in enumerate(self.tracks):
                ax, ay = a.mean_xy()
                for b in self.tracks[i + 1:]:
                    bx, by = b.mean_xy()
                    limit = min(self.cfg.assoc_radius_m,
                                max(self.cfg.merge_radius_m,
                                    OVERLAP_FACTOR * (a.spread_m() + b.spread_m())))
                    if math.hypot(ax - bx, ay - by) <= limit:
                        keep, gone = (a, b) if a.id < b.id else (b, a)
                        keep.absorb(gone)
                        self.tracks.remove(gone)
                        self._merged_into[gone.id] = keep.id
                        for old, new in self._merged_into.items():
                            if new == gone.id:
                                self._merged_into[old] = keep.id
                        merged = True
                        break
                if merged:
                    break

    def buoys(self, now=None):
        """The map: one dict per buoy with at least min_sightings sightings.

        Tracks below that stay internal. A single stray box -- a bag, a person,
        one misfire -- must not appear on a map someone will navigate by.
        """
        out = []
        for tr in self.tracks:
            if len(tr.sightings) < self.cfg.min_sightings:
                continue
            ev = tr.evidence(self.cfg)
            if tr.locked is not None:
                state, colour = tr.locked
            else:
                state, colour = tr.decide(self.cfg, ev)
                if self.cfg.lock_state and state != STATE_UNKNOWN:
                    tr.locked = (state, colour)
            mx, my = tr.mean_xy()
            lat, lon = geo.xy_to_latlon(mx, my, self.origin)
            out.append({
                "id": tr.id, "lat": lat, "lon": lon,
                "state": state, "colour": colour, "label": label(state, colour),
                "locked": tr.locked is not None,
                "lit_fraction": ev["lit_fraction"],
                "colour_agreement": ev["colour_agreement"],
                "samples": ev["samples"],
                "flash_transitions": ev["flash_transitions"],
                "observed_s": ev["observed_s"],
                "sightings": len(tr.sightings),
                "spread_m": tr.spread_m(),
                "last_seen_age_s": (None if now is None or tr.last_t is None
                                    else max(0.0, now - tr.last_t)),
            })
        return out
