"""camera_frame — which way the gimbal camera faces, and what patch of ground it sees.

PURE. No ROS, no I/O. Shared by uav_perception.geolocate (which projects buoy
boxes) and uav_groundstation (which draws the camera's footprint on the Map
tab), so the map and the buoy positions agree on the camera's heading by
construction: there is one implementation, not two that drift.
"""
import math

YAW_MODES = ("body", "earth", "aircraft")

#: The A8 mini main stream is 1920x1080. The vertical field of view is derived
#: from the horizontal one through this, rather than measured separately.
ASPECT_H_OVER_W = 1080.0 / 1920.0


def known(x) -> bool:
    """A real number: not None, not NaN. Blank index cells arrive as either."""
    return x is not None and not (isinstance(x, float) and math.isnan(x))


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


def nadir_footprint(height_m, heading_deg, hfov_deg,
                    aspect_h_over_w=ASPECT_H_OVER_W):
    """Corners of the ground rectangle a straight-down camera sees.

    Returns [(east_m, north_m) x4] relative to the point directly below the
    camera, in image order: top-left, top-right, bottom-right, bottom-left.
    "Top" is the edge the camera heading points at, so the polygon's first edge
    is the far side of the frame.

    Nadir only, and that is the honest limit rather than a shortcut: off nadir
    the patch is a trapezoid that depends on pitch, and the mapper refuses those
    frames anyway. Callers should draw nothing when the gimbal is not at nadir.
    Returns None for an unknown input or a camera not above the ground.
    """
    if not (known(height_m) and known(heading_deg)) or height_m <= 0:
        return None
    half_w = height_m * math.tan(math.radians(hfov_deg) / 2.0)
    half_h = half_w * aspect_h_over_w
    h = math.radians(heading_deg)
    fwd = (math.sin(h), math.cos(h))          # (east, north) of image-up
    right = (math.cos(h), -math.sin(h))       # (east, north) of image-right
    corners = []
    for f, r in ((half_h, -half_w), (half_h, half_w),
                 (-half_h, half_w), (-half_h, -half_w)):
        corners.append((f * fwd[0] + r * right[0], f * fwd[1] + r * right[1]))
    return corners
