# The WHOLE buoy map, every time.
#
# Never deltas. The Task 1 design settled this: any single received map fully
# resyncs whoever reads it, so a dropped packet costs one second of staleness
# rather than a boat that is permanently missing a buoy. At Disruptive tier the
# map changes while Crusader is inside the course, which is exactly when a
# delta protocol would be most expensive to get wrong.
std_msgs/Header header

Buoy[] buoys

# Why detections were or were not used, since the node started (or the map was
# last cleared). A map that stays empty while the camera plainly sees buoys is
# explained by these, not by guessing.
uint32 detections_used
uint32 detections_partial     # box touched the frame edge
uint32 detections_low_conf
uint32 frames_rejected        # no metadata, stale pose/gimbal, off nadir,
                              # camera swinging, or too low
string last_reject_reason     # the most recent frame rejection, in words

string export_stem            # file stem of the exports for this map
