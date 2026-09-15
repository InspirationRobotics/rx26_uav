# `uav_msgs` — the typed contracts

Thirteen messages. Depends on nothing but `std_msgs`, so any container builds it cheaply.

| Message | Source | Notes |
|---|---|---|
| `GlobalPos` | `GLOBAL_POSITION_INT` | the ASV's `LatLonHead` plus `altitude_amsl`, `altitude_rel`, `climb` |
| `Attitude` | `ATTITUDE` | radians, autopilot NED axes, **unconverted** |
| `FlightState` | `EXTENDED_SYS_STATE` | `landed_state` + `valid` |
| `FcuStatus` | `HEARTBEAT` | mode, armed, system_status |
| `RcChannels` | `RC_CHANNELS` | also the `/uav/rc_override` payload |
| `Battery` | `SYS_STATUS` + `BATTERY_STATUS` | volts (trust these), amps and mAh (current sensor not yet calibrated) |
| `GpsStatus` | `GPS_RAW_INT` | fix type, satellites, HDOP, receiver accuracy |
| `FcuParams` | `PARAM_VALUE` | `BATT_LOW_VOLT`, `BATT_CRT_VOLT`, `BATT_CAPACITY`, `FENCE_ENABLE`, `FENCE_ALT_MAX`, READ BACK from the autopilot; latched |
| `CameraStatus` | `camera_node` | stream, recording, gimbal pitch/yaw, and the main-stream encoding read from the camera |
| `BuoyDetection`, `BuoyDetections` | `detector_node` | one frame's boxes, with that frame's pose and gimbal angles |
| `Buoy`, `BuoyMap` | `buoy_mapper` | the whole Task 1 map, every time |

## Why separate topics and not one

`GLOBAL_POSITION_INT`, `ATTITUDE`, `EXTENDED_SYS_STATE` and `HEARTBEAT` are
**separate MAVLink streams on separate rate groups**, and they die independently.
Folding them into one message lets one stream's staleness silently gate the
others', and the consequences differ sharply: a stale mode string costs a greyed
readout, while a stale `landed_state` must **suppress the OCS heartbeat
entirely**, because RoboNation refuses `FLIGHT_PHASE_UNKNOWN` from a UAV.

The cost is that a consumer checks several topics. That cost is the point.

## `FlightState.valid`

`LANDED_STATE_UNDEFINED` is `0` and `LANDED_STATE_ON_GROUND` is `1` — one apart,
opposite in meaning. `valid` is a byte spent so a consumer cannot read the zero
as a state.

## Change impact

| You changed | Re-run |
|---|---|
| added a `.msg` | add it to `CMakeLists.txt` too — one that is missing generates nothing, the build stays green, and the import fails at node start |
| anything at all | rebuild, then restart EVERY node that uses the message, not only the one you edited: a node still running the old generated class stops talking to one running the new, silently. `grep -rn MessageName` across all packages finds them |
| `GlobalPos` altitude fields | `tools/bench/bench_heartbeat.py`, then check the Telemetry tab's three altitude cards |
