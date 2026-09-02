# Health and recording state of the gimbal camera pipeline.
#
# WHY THIS CARRIES frame_index. Video and ROS data are recorded into separate
# files by separate writers, and afterwards someone has to say which pose the
# aircraft held when a given frame was captured. Putting the frame counter on a
# topic that is itself recorded into the bag means the bag alone is enough to
# align them: no clock offset to estimate, no assumption that the two writers
# started at the same instant. The sidecar _frames.csv carries the same counter,
# so the CSV and the bag join on a number both of them wrote down.
#
# Publish rate is the node's status period, NOT the frame rate. This topic is a
# heartbeat about the pipeline, not a per-frame record.
std_msgs/Header header

# ---------------------------------------------------------------- stream

# True when a frame arrived within the node's frame_timeout_s. False is the
# honest answer to "is video flowing", and it is published rather than the node
# simply going quiet, because the ground station needs to distinguish "camera
# node is dead" from "camera node is alive and the camera is not".
bool stream_ok

# Measured over the last status period. NaN before the first measurement.
# NaN is real data here: it means "not yet known", which is different from 0.0,
# which would mean "measured, and nothing is arriving".
float64 fps

# Since the pipeline last started, not since boot. A reconnect resets them, so
# a rising frames_dropped is always about the CURRENT stream.
uint64 frames_total
uint64 frames_dropped

# ---------------------------------------------------------------- recording

# Writing the .mkv on the Jetson.
bool recording_local

# The camera's own SD card, commanded over the SIYI SDK. Separate field because
# the two record paths fail independently and that is the entire reason both
# exist: a Jetson crash must not cost the footage, and a full SD card must not
# stop the flight log.
bool recording_sd

# UTC stem shared by <session>.mkv, <session>.bag and <session>_frames.csv.
# Empty string when not recording. Kept as a string rather than reconstructed
# from header.stamp because the stem is a FILENAME — whoever reads this topic
# to find the footage needs the exact characters, not a timestamp to re-format.
string session

# Index of the newest frame written to _frames.csv. THE JOIN KEY (see header).
uint64 frame_index

# ---------------------------------------------------------------- resources

# Free space on the recording filesystem. Published so a full disk is visible
# at the ground station BEFORE it stops the recording, rather than being
# discovered afterwards in an empty file.
float64 disk_free_mb

# ---------------------------------------------------------------- gimbal

# MEASURED gimbal pitch in degrees, not the commanded value. Nadir is -90.
#
# The distinction matters downstream: geo-projection turns a pixel into a
# lat/lon using this angle, and a gimbal that is commanded to -90 but sitting at
# -87 puts every projected position out by altitude*tan(3deg) — about 1 m at
# 20 m. Publishing the command instead would hide exactly the error the
# projection cannot tolerate.
#
# NaN when the gimbal has not answered. Do not substitute the commanded angle.
float64 gimbal_pitch

# False when the gimbal link is not answering. gimbal_pitch is NaN whenever this
# is false, but the flag is carried explicitly so a consumer cannot read a NaN
# as a number.
bool gimbal_ok
