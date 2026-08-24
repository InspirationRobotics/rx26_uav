# `uav_bringup` — launch + params

Ships no code. It is the build entry point: `--packages-up-to uav_bringup` walks
this package's `exec_depend` list, so **a package missing from `package.xml`
silently stops being built** and surfaces as an import error at node start.

```bash
ros2 launch uav_bringup core.launch.py
```

## What the launch starts, and what it does not

`telemetry_bridge` only. `ground_station` and `ocs_client` have their own systemd
units instead — a launch file is one process tree, so a crash-looping display
would take the telemetry gateway down with it on every restart. Separate units
also mean `systemctl disable` works on either without editing this file.

## `config/uav_params.yaml`

The single source of truth. Node code loads it for declaration defaults, so a
code default cannot drift from the file the launch system passes.

**Two rcl format rules.** Break either and every node dies at `rclpy.init()`
before a line of node code runs — PyYAML accepts both, which is exactly why a
human reading the file does not catch them:

1. **No anchors/aliases.** `rcl_yaml_param_parser` is token-level and rejects
   them outright.
2. **Every top-level key is a node name whose only child is `ros__parameters:`** —
   including `shared`, which is documentation that no node loads. It carries that
   level only because rcl rejects a top-level scalar.

So values that must stay equal are written out **literally**, several times, and
`tools/scripts/check_config.py` is the only thing that notices when one drifts.

| Duplicated | Copies |
|---|---|
| `pose_timeout_s` | 4 — `shared`, bridge `stream_timeout_s`, `ocs_client`, `ground_station` |
| `geoid_separation_m` | 2 — `shared`, `ocs_client` |
| **`geofence`** | 3 here, **plus `uav_geofence` in `rx26_ocs/bridge.toml`** |

The geofence is the one that matters most: if our copies and the OCS's disagree,
we upload one polygon to the autopilot and declare a different one to
RoboNation — flying a box we did not declare. `check_config.py` compares across
repos when `rx26_ocs` is checked out beside this one.

## Two values that are placeholders

Both are marked in the file, and both are silently wrong rather than loudly
broken:

- **`geofence`** — a ~200 m box at the Singapore coordinates RoboNation's
  `test_server` publishes. Realistic, not the competition fence. Replace it in
  **both repos, together.**
- **`geoid_separation_m`** — `-24.6` is Sarasota. Singapore differs. Check it
  against a known field elevation before the competition.

## Change impact

| You changed | Re-run |
|---|---|
| any pinned value | `python3 tools/scripts/check_config.py` |
| the geofence | check_config **with `rx26_ocs` beside this repo**, so the cross-repo comparison runs; then re-upload and diff in QGC vertex for vertex |
| added a package | add it to `package.xml` exec_depends, or it stops being built |
| a param's range | it lives in the node's `PARAM_SPEC`; a disagreement means a stale install space, and the error text says so |
| ports | `check_config.py` — it fails if `mav_endpoint` drifts into the ASV's 1455x range |
