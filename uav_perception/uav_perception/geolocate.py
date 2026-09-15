"""geolocate — a pixel in a camera frame -> a latitude/longitude on the surface.

PURE. No ROS, no clock, no I/O, so the live mapper and the offline replay tool
run exactly the same arithmetic, and a bench can check it with numbers worked
out by hand.

THE MODEL, in the order the arithmetic runs:

  1. PIXEL -> RAY. A pinhole camera, principal point at the image centre, focal
     length from the MEASURED horizontal field of view (81 deg on the A8 mini:
     1130 px at 1920 wide). Taken from the frame's actual width every time, so
     a stream that falls back to 720p cannot silently use a 1080p focal length.

  2. RAY -> TILTED BY THE GIMBAL. The gimbal's MEASURED pitch (nadir = -90), not
     the commanded one: a camera commanded to -90 but sitting at -87 pushes every
     position out by height*tan(3 deg). The aircraft's own roll and pitch are
     NOT applied, because the gimbal exists to cancel them; its measured pitch
     is already the camera's attitude relative to the horizon.

  3. RAY -> HITS THE SURFACE at `height_m` below the camera: altitude above home,
     plus how high home sits above the surface (0 at the park, the dock height on
     water), minus the height of the buoy top the box outlines.

  4. OFFSET -> ROTATED TO NORTH/EAST by the CAMERA's heading. Image-up is the
     direction the camera faces; image-right is 90 deg clockwise of it. The
     camera heading is the aircraft heading plus the gimbal's yaw plus a fixed
     mount offset -- see camera_heading_deg for why each term is there.

  5. METRES -> DEGREES with uav_common.geo, the fleet's one equirectangular
     conversion. A buoy the boat places and a buoy this places must use the same
     constant, or they disagree by more than the buoys are apart.

WHAT IT IGNORES, deliberately and measurably small at 10 m and hover speed:
lens distortion (and sightings near the image centre are weighted up, where it
is least), the GPS antenna's offset from the camera (a few centimetres on this
frame), and curvature.

Frames that should not be projected at all are refused in words by
Projector.reject_reason; see there for each gate and what it protects against.
"""
import math

from uav_common import geo


def known(x) -> bool:
    """A real number: not None, not NaN. Blank index cells arrive as either."""
    return x is not None and not (isinstance(x, float) and math.isnan(x))


def focal_px(width_px: float, hfov_deg: float) -> float:
    """Focal length in pixels for a frame `width_px` wide."""
    if width_px <= 0 or not 0.0 < hfov_deg < 180.0:
        raise ValueError("need width > 0 and 0 < hfov < 180, got %r, %r"
                         % (width_px, hfov_deg))
    return (width_px / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


YAW_MODES = ("body", "earth", "aircraft")


def camera_heading_deg(aircraft_yaw_rad, gimbal_yaw_deg, mode, sign,
                       mount_offset_deg):
    """The direction the top of the image faces, degrees clockwise from north.

    mode:
      "body"     gimbal yaw is measured RELATIVE TO THE AIRFRAME (what follow
                 mode reports on most SIYI units). heading = aircraft yaw +
                 sign*gimbal yaw + offset. This is the term that matters in a
                 turn: the gimbal trails a fast yaw, so for a moment the camera
                 is NOT looking where the nose points, and the aircraft heading
                 alone would put every buoy on an arc.
      "earth"    gimbal yaw is already an absolute heading. heading = sign*gimbal
                 yaw + offset.
      "aircraft" ignore the gimbal's yaw. For recordings made before the index
                 carried it, and as a fallback if the gimbal's number turns out
                 to mean something else.

    sign is +1 or -1, because the SIYI protocol's yaw direction is a property of
    the unit and has to be checked once on the bench (turn the aircraft by hand;
    see the field-day skill), not assumed.

    mount_offset_deg is the fixed angle between where the gimbal thinks "zero"
    is and where the nose actually points. Measured once, never tuned in flight.

    Returns None when an input the mode needs is unknown: no guessing.
    """
    if mode not in YAW_MODES:
        raise ValueError("gimbal_yaw_mode must be one of %s, got %r"
                         % (YAW_MODES, mode))
    if mode == "earth":
        if not known(gimbal_yaw_deg):
            return None
        h = sign * gimbal_yaw_deg + mount_offset_deg
    else:
        if not known(aircraft_yaw_rad):
            return None
        h = math.degrees(aircraft_yaw_rad) + mount_offset_deg
        if mode == "body":
            if not known(gimbal_yaw_deg):
                return None
            h += sign * gimbal_yaw_deg
    return h % 360.0


def ground_offset(u, v, width, height, f_px, height_m, cam_pitch_deg,
                  cam_heading_deg):
    """Pixel (u, v) -> (north_m, east_m) from the point directly below the camera.

    Returns None when the ray does not come down to the surface in front of the
    camera -- a pixel above the horizon of a camera pitched well up. That cannot
    happen near nadir, and a None forces the caller to handle it rather than
    receive a position hundreds of metres out.

    Frame (F, R, D) = forward along the camera heading, right, down.
      optical axis    z = ( cos p, 0, -sin p)   p = camera pitch; -90 -> straight down
      image right     x = ( 0,     1,  0    )
      image down      y = ( sin p, 0,  cos p)   = z cross x; at nadir, image-down
                                                  points BACKWARD, so image-up is
                                                  the direction the camera faces
    """
    if height_m <= 0:
        return None
    p = math.radians(cam_pitch_deg)
    dx = (u - width / 2.0) / f_px
    dy = (v - height / 2.0) / f_px
    d_f = math.cos(p) + dy * math.sin(p)
    d_r = dx
    d_d = -math.sin(p) + dy * math.cos(p)
    if d_d <= 1e-6:
        return None
    t = height_m / d_d
    fwd, right = t * d_f, t * d_r
    h = math.radians(cam_heading_deg)
    north = fwd * math.cos(h) - right * math.sin(h)
    east = fwd * math.sin(h) + right * math.cos(h)
    return north, east


def centre_weight(u, v, width, height) -> float:
    """How much one sighting counts toward the buoy's averaged position.

    Heading, height and distortion errors all grow with distance from the image
    centre (a 5 deg heading error is ~0.75 m at the frame edge from 10 m and
    nothing at the centre), so a sighting from overhead is worth several from the
    edge. 4.0 at the centre, 0.8 at the corner.
    """
    half_diag = math.hypot(width / 2.0, height / 2.0)
    r = math.hypot(u - width / 2.0, v - height / 2.0) / half_diag
    return 1.0 / (0.25 + r * r)


class Projector:
    """The mapper's geometry settings, and the two questions asked of each frame.

    Every setting is a MEASURED constant or a safety gate. None of them is meant
    to be adjusted in flight: if a map comes out wrong, the replay tool re-runs
    the recorded sightings with a corrected constant after landing.
    """

    def __init__(self, *, hfov_deg=81.0, nadir_pitch_deg=-90.0,
                 max_off_nadir_deg=8.0, gimbal_yaw_mode="body",
                 gimbal_yaw_sign=1.0, mount_yaw_offset_deg=0.0,
                 launch_height_above_surface_m=0.0, target_height_m=0.45,
                 min_alt_m=3.0, max_pose_age_s=0.5, max_gimbal_age_s=0.5,
                 max_gimbal_yaw_rate_dps=20.0):
        if gimbal_yaw_mode not in YAW_MODES:
            raise ValueError("gimbal_yaw_mode must be one of %s, got %r"
                             % (YAW_MODES, gimbal_yaw_mode))
        if gimbal_yaw_sign not in (1, -1, 1.0, -1.0):
            raise ValueError("gimbal_yaw_sign must be +1 or -1, got %r"
                             % (gimbal_yaw_sign,))
        self.hfov_deg = float(hfov_deg)
        self.nadir_pitch_deg = float(nadir_pitch_deg)
        self.max_off_nadir_deg = float(max_off_nadir_deg)
        self.gimbal_yaw_mode = gimbal_yaw_mode
        self.gimbal_yaw_sign = float(gimbal_yaw_sign)
        self.mount_yaw_offset_deg = float(mount_yaw_offset_deg)
        self.launch_height_above_surface_m = float(launch_height_above_surface_m)
        self.target_height_m = float(target_height_m)
        self.min_alt_m = float(min_alt_m)
        self.max_pose_age_s = float(max_pose_age_s)
        self.max_gimbal_age_s = float(max_gimbal_age_s)
        self.max_gimbal_yaw_rate_dps = float(max_gimbal_yaw_rate_dps)

    def reject_reason(self, f):
        """None if frame `f` may be projected, else why not, in words.

        `f` is a mapping with recorder_core.FRAME_FIELDS names (lat, lon,
        alt_rel, yaw, pose_age_s, gimbal_pitch, gimbal_yaw, gimbal_yaw_rate,
        gimbal_age_s). Unknown values are None or NaN.

        Each gate is a way a correct-looking position comes out wrong:
          stale pose    the aircraft has moved since the fix it would be placed from
          stale gimbal  the camera angle is a remembered one, not a measured one
          off nadir     the gimbal is still slewing back (after a power cycle or a
                        hard manoeuvre) and height*tan(error) grows fast
          swinging      the camera is mid-turn, so its heading changed during the
                        few tens of ms between the frame and its gimbal sample
          too low       take-off and landing: the ground is metres away, buoys
                        are huge and clipped, and the barometer is least reliable
        """
        for name in ("lat", "lon", "alt_rel"):
            if not known(f.get(name)):
                return "no pose for this frame"
        if not known(f.get("pose_age_s")) or f["pose_age_s"] > self.max_pose_age_s:
            return "pose too old"
        if f["alt_rel"] < self.min_alt_m:
            return "below min_alt_m"
        g_pitch, g_age = f.get("gimbal_pitch"), f.get("gimbal_age_s")
        if not known(g_pitch) or not known(g_age) or g_age > self.max_gimbal_age_s:
            return "no fresh gimbal reading"
        if abs(g_pitch - self.nadir_pitch_deg) > self.max_off_nadir_deg:
            return "gimbal off nadir"
        if self.gimbal_yaw_mode != "aircraft":
            rate = f.get("gimbal_yaw_rate")
            if known(rate) and abs(rate) > self.max_gimbal_yaw_rate_dps:
                return "camera still turning"
        if self.heading_deg(f) is None:
            return "no camera heading"
        return None

    def heading_deg(self, f):
        return camera_heading_deg(f.get("yaw"), f.get("gimbal_yaw"),
                                  self.gimbal_yaw_mode, self.gimbal_yaw_sign,
                                  self.mount_yaw_offset_deg)

    def surface_height_m(self, f) -> float:
        return (f["alt_rel"] + self.launch_height_above_surface_m
                - self.target_height_m)

    def locate(self, f, u, v, width, height):
        """-> (lat, lon, weight) for pixel (u, v), or None. Call reject_reason first."""
        off = ground_offset(u, v, width, height,
                            focal_px(width, self.hfov_deg),
                            self.surface_height_m(f), f["gimbal_pitch"],
                            self.heading_deg(f))
        if off is None:
            return None
        north, east = off
        lat, lon = geo.xy_to_latlon(east, north, (f["lat"], f["lon"]))
        return lat, lon, centre_weight(u, v, width, height)
