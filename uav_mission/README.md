# `uav_mission` — the aircraft doing a task on its own

```bash
ros2 run uav_mission search_node      # or Start on the ground station's Nodes tab
```

One node, `search_node`: the **Task 1 buoy search** (Advanced tier). No systemd
unit; start it from the Nodes tab with `detector_node` and `buoy_mapper`.

**Tested in ArduCopter 4.7.0 SITL** with the real `telemetry_bridge`,
`ground_station` and this node (a stand-in for the camera and mapper). Not yet
flown.

## What it does

1. Sweeps the area inside **the fence the autopilot holds** (read back by
   `telemetry_bridge`, so at practice it is whatever was drawn in QGC) at
   10 m, in lines 10 m apart, keeping every point it flies to 2 m inside the
   fence. The nose holds one heading for a whole pass; the aircraft flies the
   return lines backwards, so the gimbal never swings.
2. The moment the buoy map has an **UNKNOWN** buoy inside the fence, it flies
   over it and hovers until `buoy_mapper` locks its state. Unknown buoys within
   4 m of each other (a 3 m gate) share one hover.
3. Confirmed: it flies back to where it left the line and carries on.
4. **Not confirmed after 10 s overhead:** given up on for the rest of that pass.
5. A pass that ends short of the count starts another with the lines turned
   90°; every other pair of passes also shifts them half a line.
6. **Once the ground station's "buoys to find" are confirmed inside the fence,
   it asks for RTL.**

## Who is flying

- **Only the pilot starts it**: on a *change* into GUIDED (the SC switch), while
  switched on from the page and ready. Being in GUIDED already is not a start,
  so switching it on from the page, or restarting the node in the air, never
  moves the aircraft.
- **Any other mode pauses it** — SB to Loiter or Brake, SC off, an RTL — with
  its place kept. SC off and on again resumes it.
- **Switched off from the page** while in GUIDED: it holds position, and needs
  the SC switch again to resume.
- **Buoy map lost** while flying: it holds, and resumes by itself when the map
  returns.
- **The node dies**: the autopilot finishes the leg it was on (inside the fence,
  at 10 m) and holds there in GUIDED until the pilot flips a switch.
- The only mode it asks for is RTL, and only from GUIDED.

`telemetry_bridge` checks every target again before it reaches the autopilot —
GUIDED and armed, recent, inside the fence, under `FENCE_ALT_MAX - FENCE_MARGIN`
— so a bug here can ask for a bad point but not fly one.

## What it needs before it will start

The page's Map tab and the checklist say which of these is missing, in words.

| | |
|---|---|
| Autopilot | armed, in the air, GPS position |
| Fence | one polygon, no exclusion zones or circles, **no inward corners** (a vertex within 0.75 m of straight is fine), read from the autopilot in the last 90 s |
| `FENCE_ENABLE` 1, `FENCE_TYPE` with the polygon bit | the fence must actually be enforced |
| `FENCE_ALT_MAX - FENCE_MARGIN` ≥ 10 m | with the altitude bit set. Saturday setup: 12 and 2 |
| Buoy map | `detector_node` and `buoy_mapper` running |
| The count | not already met by the map. **Buoys confirmed on an earlier flight count** — clear the buoys on the Map tab before a fresh run |

The convex-fence rule is deliberate: inside a convex polygon every straight leg
stays inside, so there is no path planner to get wrong. See `sweep_core.py`.

## Files

| | |
|---|---|
| `sweep_core.py` | fence shrinking, sweep lines, waypoint order. Pure |
| `search_core.py` | the state machine and every rule above. Pure |
| `search_node.py` | ROS wiring only |

`tools/bench/bench_search.py` drives both cores, and the bridge's gate, with no
aircraft.

## Parameters

In `uav_params.yaml` under `search_node`. Two are set from the page and are the
only dynamic ones: `enabled`, `buoys_to_find`. The rest are read-only.
`search_alt_m` is pinned to `buoy_mapper.waypoint_alt_m` by `check_config.py`.
