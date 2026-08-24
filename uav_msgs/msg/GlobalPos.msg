# Position, heading and altitude sample from the autopilot's GLOBAL_POSITION_INT.
#
# The ASV's LatLonHead with the three fields an aircraft cannot do without. They
# cost nothing extra: alt, relative_alt and vz all arrive in the SAME MAVLink
# frame as the latitude and longitude, so carrying them here is free and
# finite-differencing position for a climb rate is re-deriving a number the
# autopilot already measured.
std_msgs/Header header
float64 latitude
float64 longitude

# Degrees, 0 = true north, clockwise positive.
#
# NaN when the autopilot reports hdg == 65535, i.e. GPS yaw is unresolved. That
# is REAL DATA and consumers must not scrub it: the OCS validator exists to
# refuse a heartbeat carrying it, and replacing it with a plausible number here
# would hide a fault at the one place it is still cheap to notice.
float64 heading

# Horizontal ground speed [m/s], from vx/vy. Horizontal only, because the
# RxReport heartbeat's spd_mps is horizontal; vertical motion is `climb` below,
# which is a separate field for a separate question.
float64 ground_speed

# Altitude above MEAN SEA LEVEL [m], from GLOBAL_POSITION_INT.alt.
#
# NOT height above the WGS84 ellipsoid, which is what RoboNation's
# Heartbeat.altitude_hae_m wants. The two differ by the geoid separation, which
# is tens of metres and venue-specific (~ -24.6 m at Sarasota, different at
# Singapore). ocs_client adds `geoid_separation_m` on the way out; nothing else
# may assume these are the same number.
float64 altitude_amsl

# Altitude above the HOME position [m], from GLOBAL_POSITION_INT.relative_alt.
# This is the one a pilot reads and the one the airborne/grounded fallback in
# ocs_client uses. Meaningless before home is set.
float64 altitude_rel

# Climb rate [m/s], POSITIVE UP. MAVLink vz is positive DOWN (NED), so this is
# negated on the way in by uav_common.geo.climb_rate_mps — the sign flip lives
# in one tested place rather than being re-derived, wrongly, by a second reader.
float64 climb
