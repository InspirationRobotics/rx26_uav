"""recorder_core — session naming, the frame index, and the disk guard.

WHAT THIS FILE EXISTS TO PREVENT. A sortie produces three artifacts written by
three different writers: the video (GStreamer), the ROS bag (rosbag2), and the
frame index (this node). Afterwards someone has to answer "where was the
aircraft when frame 4127 was captured", and they have to answer it months later,
from files alone, with no memory of the flight.

Every cheap way of answering that is wrong:

  * Matching on file modification time assumes the two writers started together.
    They do not — GStreamer negotiates with the camera first.
  * Interpolating pose to the frame timestamp assumes pose was arriving. When
    MAVProxy drops, /uav/pose stops, and an interpolation across that gap
    invents a position for exactly the frames that have none.
  * Storing "the last pose we saw" alongside the frame loses the fact that it
    was 900 ms old, which is the one thing that decides whether the frame is
    usable as training data.

So the index records the join key (ros_time_ns, the same clock the bag stamps),
the age of the pose at the moment the frame arrived, and BLANKS rather than
stale values when the pose was too old. A blank is a fact. A back-filled
coordinate is a guess that looks like a measurement, and six months later
nothing distinguishes it from one.

This is the same rule uav_common.stream_cache enforces on the wire, applied to
what gets written to disk.

PURE. No ROS, no clock, no filesystem. `now` and `when` are supplied by the
caller so a bench drives time deterministically.
"""

# Column order is part of the file format. Anything already labelled was written
# against this order; appending is safe, reordering or inserting is not.
FRAME_FIELDS = (
    "frame_idx",     # monotonic from 0 at session start; the bag join key
    "pts_ns",        # presentation timestamp in the video container
    "ros_time_ns",   # ROS clock at frame receipt; joins to the bag
    "lat",           # blank when the pose was stale -- see module docstring
    "lon",
    "alt_rel",       # metres above home
    "roll",          # radians, autopilot axes (see uav_msgs/Attitude.msg)
    "pitch",
    "yaw",
    "gimbal_pitch",  # degrees, MEASURED, nadir is -90. Blank when unknown
    "pose_age_s",    # age of the pose at frame receipt; blank if none ever seen
    # ---- appended 2026-09-13 for buoy geolocation. APPENDED, not inserted, so
    # every index written before this still parses column-for-column.
    #
    # The camera's heading is NOT the aircraft's. In follow mode the gimbal
    # trails a fast yaw and catches up a moment later, so projecting a pixel
    # with the aircraft heading alone puts buoys in the wrong place exactly
    # while the pilot is turning. The gimbal reports where it is; record it.
    "gimbal_yaw",       # degrees, as the gimbal reports it. Blank when unknown
    "gimbal_yaw_rate",  # degrees/s; large = the camera is still swinging round
    "gimbal_age_s",     # age of the gimbal sample at frame receipt
)

# How each column is written. Kept beside FRAME_FIELDS so a new column cannot be
# added without saying how it is formatted.
FIELD_FORMAT = {
    "lat": ".7f", "lon": ".7f",   # 7 dp is ~11 mm: below the GPS's own error
    "alt_rel": ".3f",
    "roll": ".6f", "pitch": ".6f", "yaw": ".6f",
    "gimbal_pitch": ".2f", "gimbal_yaw": ".2f", "gimbal_yaw_rate": ".1f",
    "pose_age_s": ".3f", "gimbal_age_s": ".3f",
}

# Stems sort lexicographically into chronological order, which is the only
# property that matters when someone is looking for "the flight after lunch" in
# a directory listing on a field laptop.
STEM_FORMAT = "%Y%m%dT%H%M%SZ"


def session_stem(when) -> str:
    """UTC datetime -> the stem shared by every artifact of one sortie.

    `when` must already be UTC. Local time is refused rather than converted:
    this team flies in California and competes in Singapore, and a stem that
    silently means one or the other depending on where the laptop was is worse
    than no stem at all.
    """
    if when.tzinfo is None:
        raise ValueError(
            "session_stem() needs an aware UTC datetime; got a naive one. "
            "Use datetime.now(timezone.utc), not datetime.now().")
    if when.utcoffset().total_seconds() != 0:
        raise ValueError(
            f"session_stem() needs UTC; got offset {when.utcoffset()}. "
            "Convert with .astimezone(timezone.utc) before calling.")
    return when.strftime(STEM_FORMAT)


def csv_header() -> str:
    """First line of <stem>_frames.csv, without a trailing newline."""
    return ",".join(FRAME_FIELDS)


def _fmt(value, spec):
    """A number formatted to `spec`, or "" for None.

    The empty field is load-bearing: pandas and csv both read it as missing
    rather than as a value, so a stale-pose frame cannot be silently averaged
    into a training set.
    """
    return "" if value is None else format(value, spec)


def frame_values(frame_idx, pts_ns, ros_time_ns, *, pose=None, attitude=None,
                 gimbal_pitch=None, pose_age_s=None, gimbal_yaw=None,
                 gimbal_yaw_rate=None, gimbal_age_s=None) -> dict:
    """Every FRAME_FIELDS value for one frame, None where unknown.

    `pose` is (lat, lon, alt_rel) or None when the pose cache was stale.
    `attitude` is (roll, pitch, yaw) or None, independently -- the two arrive on
    separate MAVLink streams with separate rate groups and they go stale
    separately, which is the same reason uav_msgs keeps GlobalPos and Attitude
    as different messages.

    ONE FUNCTION FEEDS BOTH the CSV row and the metadata camera_node stamps onto
    the viewer stream. The live buoy mapper reads the stream; the replay tool
    reads the CSV. They must see the same numbers for the same frame, and two
    code paths that "compute the same thing" are how that stops being true.

    Passing a stale value here instead of None defeats the entire file. There is
    no parameter to turn that behaviour on.
    """
    lat, lon, alt = pose if pose is not None else (None, None, None)
    roll, pitch, yaw = attitude if attitude is not None else (None, None, None)
    return {
        "frame_idx": int(frame_idx), "pts_ns": int(pts_ns),
        "ros_time_ns": int(ros_time_ns),
        "lat": lat, "lon": lon, "alt_rel": alt,
        "roll": roll, "pitch": pitch, "yaw": yaw,
        "gimbal_pitch": gimbal_pitch, "pose_age_s": pose_age_s,
        "gimbal_yaw": gimbal_yaw, "gimbal_yaw_rate": gimbal_yaw_rate,
        "gimbal_age_s": gimbal_age_s,
    }


def row_from_values(values: dict) -> str:
    """frame_values() output -> one index row, without a trailing newline."""
    out = []
    for name in FRAME_FIELDS:
        v = values.get(name)
        spec = FIELD_FORMAT.get(name)
        out.append(str(int(v)) if spec is None else _fmt(v, spec))
    return ",".join(out)


def csv_row(frame_idx, pts_ns, ros_time_ns, **kwargs) -> str:
    """One index row, without a trailing newline. See frame_values()."""
    return row_from_values(frame_values(frame_idx, pts_ns, ros_time_ns,
                                        **kwargs))


class DiskGuard:
    """Stops recording before the filesystem fills, and says so exactly once.

    Recording into a full disk does not fail cleanly: the muxer cannot write its
    index, and a matroska file whose index never landed may not seek, or may not
    open at all. Losing the last minute of a sortie is acceptable; losing the
    whole file because nobody noticed the disk was full is not.

    The one-shot edge mirrors StreamCache.went_stale for the same reason -- a
    condition that persists for the rest of the flight must produce one loud
    line, not one per frame.
    """

    __slots__ = ("min_free_mb", "_tripped")

    def __init__(self, min_free_mb: float):
        if min_free_mb <= 0:
            raise ValueError("min_free_mb must be positive")
        self.min_free_mb = float(min_free_mb)
        self._tripped = False

    def check(self, free_mb: float):
        """-> (may_record, newly_tripped, reason).

        `newly_tripped` is True exactly once per crossing, for the log line.
        `reason` is empty while there is room.
        """
        if free_mb >= self.min_free_mb:
            self._tripped = False
            return True, False, ""
        newly = not self._tripped
        self._tripped = True
        return False, newly, (
            f"disk below floor: {free_mb:.0f} MB free, "
            f"{self.min_free_mb:.0f} MB required -- recording stopped")

    def reset(self):
        """Forget the trip so the next crossing logs again. For a new session."""
        self._tripped = False


def should_rotate(started_at: float, now: float, max_session_s: float) -> bool:
    """True when the current file has run long enough to roll to a new one.

    Rotation bounds what a crash costs. A sortie written as one file loses
    everything if the Jetson goes down before the muxer finalises; rotating
    every few minutes caps the loss at one segment.

    max_session_s <= 0 disables rotation, which is the right setting on a bench
    where a single continuous file is easier to work with.
    """
    if max_session_s <= 0:
        return False
    return (now - started_at) >= max_session_s
