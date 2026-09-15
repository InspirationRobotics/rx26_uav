# One mapped buoy: where it is, and which of the five Task 1 states it shows.
#
# THE STATE IS DECIDED OVER TIME, never from one frame. Task 1's beacon flashes
# 1 s on / 1 s off, so FLASHING BLUE (safe ENTRY) and SOLID BLUE (safe EXIT)
# look identical in any single photograph, and a flashing buoy caught in its
# dark second looks OFF. buoy_mapper_node watches each buoy in FULL VIEW for
# min_observe_s before it will say anything but UNKNOWN.

uint32 id                     # stable for the life of the map; never reused

float64 latitude              # deg, weighted mean of every usable sighting
float64 longitude             # deg

# "UNKNOWN" until watched long enough; then "OFF", "FLASHING" or "SOLID".
string state
# "RED", "GREEN", "BLUE", or "" when state is OFF or UNKNOWN.
string colour
# state and colour joined for display and reports: "FLASHING_BLUE",
# "SOLID_BLUE", "OFF", "UNKNOWN". Handbook meaning: FLASHING_RED = pass on the
# USV's starboard, FLASHING_GREEN = port, FLASHING_BLUE = safe ENTRY,
# SOLID_BLUE = safe EXIT, OFF = obstacle.
string label
# True once the state has been decided and frozen (Advanced tier: the passage
# does not change mid-run). Disruptive tier will clear this.
bool locked

# The evidence, so a surprising answer can be checked rather than trusted.
float32 lit_fraction          # share of full-view samples the model saw lit
float32 colour_agreement      # share of lit samples that agreed on `colour`
uint32 samples                # full-view detections with a class
uint32 flash_transitions      # lit<->dark changes seen across those samples
float32 observed_s            # full-view watch time accumulated so far
uint32 sightings              # detections that contributed to the position
float32 spread_m              # weighted RMS distance of sightings from the
                              # mean. THE SELF-CHECK: a large spread means the
                              # projection is off (heading, height, timing),
                              # and it shows up here rather than as a wrong pin.
float32 last_seen_age_s       # seconds since the last usable sighting
