# One buoy box from one frame, as the detector saw it.
#
# The class is the MODEL'S per-frame answer, and a single frame cannot tell the
# five Task 1 states apart: a flashing beacon photographed in its off second is
# indistinguishable from an unlit one. So nothing downstream may treat
# class_name as the buoy's state. buoy_mapper_node accumulates these over time
# and decides the state per buoy; see Buoy.msg.

# Model class name, exactly as the weights name it (e.g. "blue", "dark",
# "green", "red"). A string rather than an enum because the names come from the
# trained model, and a mapping written here would silently drift from the
# weights the day someone retrains with a different class order.
string class_name
uint8 class_id
float32 confidence

# Box corners in PIXELS of the frame the detector received (image_width x
# image_height on the parent message). Top-left origin, x right, y down.
float32 x0
float32 y0
float32 x1
float32 y1

# True when no side of the box is within the detector's edge margin of the
# image border. FALSE MEANS DO NOT USE IT: a buoy cut off by the frame edge
# usually has its beacon out of view, so the model reads it as "dark", and the
# centre of a clipped box is the centre of the VISIBLE part, not of the buoy.
# Both the state and the position would be wrong. Carried rather than dropped
# so the operator's view can still show what was seen.
bool full_view
