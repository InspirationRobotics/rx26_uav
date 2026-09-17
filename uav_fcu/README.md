# `uav_fcu` — the autopilot gateway

```bash
ros2 launch uav_bringup core.launch.py
```

One node, `telemetry_bridge`. The single ROS-side consumer of MAVProxy's
rebroadcast and the single sender back to it. Nothing else in the workspace may
open a MAVLink connection.

**Flown on Ekko** (ArduCopter 4.7.0, CubeOrange+).

## Four jobs

**RX** — republishes `/uav/pose`, `/uav/attitude`, `/uav/fcu_status`,
`/uav/flight_state`, `/uav/rc_channels`, `/uav/battery`, `/uav/gps`,
`/uav/autonomy_drop`, each **only while fresh**, with the stamp captured at
receipt. Also reads eight autopilot parameters back (`PARAM_REQUEST_READ`,
read-only) and publishes them latched on `/uav/fcu_params`, so the ground
station's battery and fence checks — and the buoy search — use the autopilot's
real thresholds instead of a copy in our YAML.

**RX (the fence the autopilot holds)** — reads the fence back over the mission
protocol at startup, every 15 s while disarmed (so a fence drawn in QGC shows
up), on each arm, and every 30 s in the air, and publishes it latched on
`/uav/fence`. It keeps reading while armed on purpose: the search refuses to
start on a fence older than 90 s, and a single read taken at arming that failed
used to block the search for the whole flight. Valid only
when the autopilot holds exactly one inclusion polygon and no exclusion zones or
circles; otherwise `problem` says what is there. On its own thread. Mission
frames are only taken if addressed to this node's sysid and of type FENCE, so
QGC fetching its own copy is never read as our answer.

**TX (autonomy)** — the only sanctioned RC-override path, gated by the
autonomy-drop latch. Nothing publishes to `/uav/rc_override` yet; the enforcement
point exists before anything needs it.

**TX (geofence)** — `/uav/fence_upload` (`std_srvs/Trigger`) uploads the
configured polygon and verifies the readback.

## There is no disarm path, deliberately

The ASV's version carries a force-disarm (`MAV_CMD_COMPONENT_ARM_DISARM` with the
21196 force magic) driven by RC loss. **Do not port it.** On a boat that stops
the thrusters and the hull floats; on a multirotor it stops the motors and the
aircraft falls. RC loss belongs to the autopilot's own failsafe params, which
work without a companion computer being alive to have an opinion.

## The geofence dialog

Runs on a **service callback thread**, not the RX thread — it blocks on each
autopilot reply, and the RX thread keeps every other stream alive. `MISSION_*`
frames are routed from the RX loop into a `queue.Queue` the transport drains.

`mav_source_system` is **200**, not pymavlink's default 255, because MAVProxy is
also 255. While the node only reads, the sysid is cosmetic; the moment it runs a
mission dialog it decides whether `MISSION_REQUEST_INT` replies are addressed to
us or to MAVProxy.

Refuses while armed. Never sets `FENCE_ENABLE` — enabling a fence is a deliberate
act at a ground station.

## Autopilot params this node depends on

Set in QGC on `MAV1_*` (USB = SERIAL0; `SR0_*` before ArduPilot 4.7). MAVProxy
runs `--streamrate=-1` precisely so it does not stomp them.

| Param | Why |
|---|---|
| `MAV1_POSITION` > 0 | `GLOBAL_POSITION_INT` → `/uav/pose` |
| `MAV1_EXTRA1` = 30 | `ATTITUDE` → `/uav/attitude` |
| `FENCE_TYPE` polygon bit | or the upload is NACKed |

`/uav/flight_state` needs no parameter. `EXTENDED_SYS_STATE` is in **no** stream group on any firmware, so no parameter turns it on: `telemetry_bridge` requests it itself with `SET_MESSAGE_INTERVAL` and re-asks every 30 s, because an autopilot reboot discards the request.

## Change impact

| You changed | Re-run |
|---|---|
| the fence path | `tools/bench/bench_fence.py`, then upload to a real Pixhawk and diff the polygon in QGC **vertex for vertex** |
| a published topic | `ocs_client` and `ground_station` both subscribe; `grep` before renaming |
| the staleness rule | stop MAVProxy and confirm **one** loud line per stream and that republishing stops |
| `mav_source_system` | the fence upload; the symptom of a collision is a timeout, not an error |
