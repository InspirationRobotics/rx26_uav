# `uav_fcu` — the autopilot gateway

```bash
ros2 launch uav_bringup core.launch.py
```

One node, `telemetry_bridge`. The single ROS-side consumer of MAVProxy's
rebroadcast and the single sender back to it. Nothing else in the workspace may
open a MAVLink connection.

**Untested in flight.**

## Three jobs

**RX** — republishes `/uav/pose`, `/uav/attitude`, `/uav/fcu_status`,
`/uav/flight_state`, `/uav/rc_channels`, `/uav/autonomy_drop`, each **only while
fresh**, with the stamp captured at receipt.

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

Set in QGC on `SR0_*` (USB = SERIAL0). MAVProxy runs `--streamrate=-1` precisely
so it does not stomp them.

| Param | Why |
|---|---|
| `SR0_POSITION` > 0 | `GLOBAL_POSITION_INT` → `/uav/pose` |
| `SR0_EXTRA1` = 30 | `ATTITUDE` → `/uav/attitude` |
| `SR0_EXT_STAT` > 0 | `EXTENDED_SYS_STATE` → `/uav/flight_state`, the authoritative `flight_phase` |
| `FENCE_TYPE` polygon bit | or the upload is NACKed |

## Change impact

| You changed | Re-run |
|---|---|
| the fence path | `tools/bench/bench_fence.py`, then upload to a real Pixhawk and diff the polygon in QGC **vertex for vertex** |
| a published topic | `ocs_client` and `ground_station` both subscribe; `grep` before renaming |
| the staleness rule | stop MAVProxy and confirm **one** loud line per stream and that republishing stops |
| `mav_source_system` | the fence upload; the symptom of a collision is a timeout, not an error |
