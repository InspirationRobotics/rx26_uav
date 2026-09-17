# The fence THE AUTOPILOT HOLDS, read back from it over the mission protocol.
#
# Not uav_params.yaml's `geofence`: that is what we would UPLOAD, and at a
# practice field the fence is drawn in QGC instead. The buoy search sweeps the
# area inside this polygon, so it must be the one the autopilot enforces --
# a sweep planned from a different polygon is a sweep that ends in a breach.
#
# telemetry_bridge reads it at startup, every few seconds while disarmed (so an
# edit in QGC shows up), and once more straight after arming. Latched
# (TRANSIENT_LOCAL). header.stamp is WHEN IT WAS READ: a consumer that needs a
# fence as of this flight compares that against the arm time.

std_msgs/Header header

# True when the autopilot holds exactly one inclusion polygon and nothing this
# vehicle's code does not understand. False otherwise, with `problem` saying why.
bool valid
string problem

# The inclusion polygon, OPEN (the autopilot closes it implicitly), in the order
# the autopilot stores it. Empty unless valid.
float64[] latitude
float64[] longitude

uint16 item_count     # every fence item the autopilot returned, of any kind
