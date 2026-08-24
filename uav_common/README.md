# `uav_common` — shared plumbing, no nodes

May not import a domain package. Contains no vehicle policy.

| Module | Purpose |
|---|---|
| `config.py` | loads `uav_params.yaml` for declaration defaults |
| `node_main.py` | `run_node` — the one entry point; converts SIGTERM into clean teardown |
| `param_utils.py` | declaration + range validation; every param gets an explicit posture |
| `stream_cache.py` | a last-known value that knows how old it is |
| `drop_latch.py` | the autonomy-drop state machine (releases overrides; **never disarms**) |
| `geo.py` | equirectangular geodesy, climb-rate sign, point-in-polygon |
| `fence_core.py` | the geofence upload protocol, with mandatory readback verify |

## Three things not to "simplify"

**`config.py`'s resolution order.** Do not replace it with
`Path(__file__).parents[N]`. Installed, this module is in
`uav_common/lib/python3.10/site-packages/` while the params are in
`uav_bringup/share/` — siblings under different packages, no fixed parent count
reaches it. That exact bug killed every ASV node at `rclpy.init()`.

**`StreamCache` has no bare accessor.** Every read passes a timestamp and gets
`None` when stale. Making the stale case unrepresentable is the point: the
original defect was a plain attribute that read the same at 20 ms and 20 minutes.

**`geo.climb_rate_mps`'s negation.** MAVLink `vz` is positive *down*. The sign
flip lives in one place because re-derived at each call site it reads correctly,
plots upside down, and is noticed on a descent.

## `fence_core`

Ported from the ASV's keep-out uploader (`rx26_asv@8c4ffa5`), changed from
circle-exclusion to **polygon inclusion** (`MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION`,
5001). Every item carries the same `param1`: the polygon's total vertex count.

`items_from_polygon` **strips the closing vertex**. The fence is stored closed
because the OCS declaration requires it; ArduPilot closes polygons implicitly, so
uploading the duplicate makes a degenerate final edge. One place converts between
the two spellings.

## Change impact

| You changed | Re-run |
|---|---|
| `fence_core.py` | `tools/bench/bench_fence.py` — 7 cases including every readback failure |
| `geo.py` | `tools/bench/bench_heartbeat.py` and the Map tab's inside/outside readout |
| `config.py` | run a node from the **install space**, not the source tree; that is where the path bug lives |
| `stream_cache.py` | pull the MAVLink stream and confirm republishing **stops** and logs once |
