# RobotX 2026 — UAV Fleet

## General information

Team Inspiration / ASTA (Advancing Science Technology and Art) aircraft for the
**RobotX 2026** system-of-systems challenge. We are aiming to complete all
missions at **disruptive tier**.

| | |
|---|---|
| RobotX website | https://robotx.org/programs/2026/ |
| RobotX rulebook | https://robonation.gitbook.io/robotx-2026-team-handbook |
| UUV repo | https://github.com/InspirationRobotics/robotx_graey_2026 |
| USV repo | https://github.com/InspirationRobotics/rx26_asv/tree/clean_usv |
| OCS repo | https://github.com/InspirationRobotics/rx26_ocs |
| UAV repo | this one (`rx26_uav`) |

The fleet is **three aircraft** — `Don`, `Ekko`, `Fitz` — of slightly different
configuration. This file is the hardware record; autopilot parameters live on
the flight controllers and in Mission Planner / QGroundControl backups, not
here.

> **Provenance.** Don's entries come from *Appendix B — Don CURRENT
> CONFIGURATION (Tattu 6000)*, the Garuda Robotics UAV Technical Information
> Form prepared 27 Aug 2026. Fitz's entries are read directly from onboard
> dataflash logs `00000064.BIN` / `00000066.BIN`. Anything marked **TBC** has
> not been confirmed by either source.

---

## Fleet at a glance

| | **Don** | **Ekko** | **Fitz** |
|---|---|---|---|
| **Role** | Competition aircraft — payload + gripper | Software development airframe *(inferred: the Jetson container is `uav_ekko`; confirm)* | Second Pixhawk 1 airframe |
| **Frame** | Tarot IRON MAN 650 folding CF quad | **9IMOD IDF-17** — 760 mm, 2.0 mm CF | Tarot Iron *(650 assumed — TBC)* |
| **Flight controller** | Pixhawk **Cube Orange+** | Pixhawk **Cube Orange+** | **Pixhawk 1** (2.4.8-class, 2 MB flash) |
| **Firmware** | TBC | TBC | ArduCopter **4.7.0** (`1511f271`) |
| **GNSS / compass** | u-blox **M9N** + external magnetometer | u-blox **M9N** + external magnetometer | u-blox **M8N**, 2 compasses (int + ext) |
| **Companion computer** | Jetson Orin Nano | Jetson Orin Nano | Jetson Orin Nano |
| **ESC** | Lumenier Elite Pro 60A AM32 4-in-1 | Lumenier Elite Pro 60A AM32 4-in-1 | Lumenier Elite Pro 60A AM32 4-in-1 — **DShot600** on outputs 9–12 |
| **Motors** | MAD 4014 IPE | MAD 4014 IPE | MAD 4014 IPE |
| **Propellers** | 14 × 5.5 carbon fibre | **16 in — planned, not yet fitted** | 14 in *(pitch TBC)* |
| **Battery** | Tattu 6000 mAh 6S1P 35C | TBC — **6S max, see note** | 6S LiPo, 4200 mAh configured — **voltage monitoring only** |
| **Operating mass** | 2.99 kg (Cfg 1) / 3.36 kg (Cfg 2) | TBC | 2800 g (3485 g demonstrated) |
| **MTOM** | 4.6 kg provisional | TBC | 4.6 kg provisional |
| **RC link** | RadioMaster Pocket → RP3 ELRS, 2.4 GHz | same | same (receiver set to **SBUS** out) |
| **Telemetry** | 915 MHz FHSS, 100 mW EIRP | 915 MHz FHSS, 100 mW EIRP | TBC |
| **Payload** | SIYI A8 mini + magnet gripper | TBC | none fitted |
| **Flotation** | Closed-cell foam, electronics uppermost | TBC | TBC |
| **Status** | Flying — last flight 24 Aug 2026 | In build — 16 in props not yet fitted | Flying — logs 28–29 Aug 2026 |

**Ekko and Don share an identical electronics stack** — Cube Orange+, M9N,
Jetson Orin Nano, Lumenier ESC, MAD 4014 IPE motors, and the same RC and
telemetry links. They are *not* interchangeable aircraft, though: Ekko's frame
is 760 mm against Don's 650 mm and its props are 16 in against 14 in.

**Propulsion hardware is common across all three; only the propellers differ.**
Ekko's 16 in props are about 30 % more disc area than the 14 in, which moves
the thrust curve, the hover point and the current draw. Combined with the
larger, heavier frame, **Ekko needs its own tune — `MOT_THST_HOVER`,
`MOT_THST_EXPO` and the rate PIDs will not transfer from Don or Fitz**, and
neither will Don's endurance or hover-current figures.

> ⚠️ **The IDF-17 frame is advertised for 8S, but the Lumenier Elite Pro ESC is
> rated 2–6S.** The ESC sets the ceiling, not the frame. Ekko must stay on a
> **6S** pack unless the ESCs are changed. Do not buy an 8S pack for it.

---

## Don

The competition aircraft, and the only one with a completed Appendix B
submission.

**Airframe and mass**

| | |
|---|---|
| Make / model | Tarot IRON MAN 650 Folding Carbon Fiber Quadcopter Frame |
| Type | Multirotor (quad), modified COTS |
| Dimensions | 0.50 m L × 0.50 m W × 0.44 m H (no props); 1.02 m diagonal |
| Empty mass | 2.10 kg |
| Config 1 — no payload | 2.99 kg operating (measured) |
| Config 2 — gripper + camera | 3.36 kg operating |
| MTOM | 4.6 kg *(provisional — to be verified by flight test)* |

**Avionics and propulsion**

| | |
|---|---|
| Flight controller | Pixhawk Cube Orange+ |
| Navigation | u-blox M9N GNSS with external magnetometer |
| Companion computer | Jetson Orin Nano |
| ESC | Lumenier Elite Pro 60A 2-6S AM32 4-in-1 (fleet-common) |
| Motors | MAD 4014 IPE (fleet-common) |
| Propellers | 14 in, 5.5 pitch |
| Obstacle avoidance | None fitted — software geofence with 2 m active-avoidance margin |

**Power**

| | |
|---|---|
| Current pack | Tattu 6000 mAh 6S1P 35C — 22.2 V nominal / 25.2 V max, 133 Wh |
| Planned pack | Tattu 16000 mAh 6S1P 25C *(separate Appendix B submitted)* |
| Hover current | 16.6 A (Config 1) / 19.7 A (Config 2), against a 210 A pack rating |
| Endurance | 12 min declared; 15 min projected in Config 1 |

**Payload (Config 2)**

| Item | Mass | Mounting / power |
|---|---|---|
| Magnet gripper | 0.233 kg | Bolted to lower frame, main battery via step-down |
| SIYI A8 mini camera | 0.144 kg | Lower frame, main battery |
| Recovery light | ~0.03 kg | Organiser-supplied, water-activated |

**Links and radio**

| | |
|---|---|
| Manual control | 2.4 GHz, 250 mW (24 dBm) EIRP |
| Telemetry | 915 MHz, 20 dBm (100 mW) EIRP, FHSS 50 channels → **must move to 920–925 MHz for IMDA compliance before Singapore** |
| Video | None |
| Wi-Fi | 2.4 / 5 GHz, ~21 dBm EIRP — *TBC whether enabled in flight* |
| GCS | QGroundControl on Windows laptop over the 915 MHz link |

**Regulatory** — FAA registered, number displayed on the airframe, Remote ID
broadcast module fitted. No team member currently holds FAA Part 107; flights
to date supervised by a certificated remote pilot.

**Repair history** — see [Fleet maintenance notes](#fleet-maintenance-notes).

---

## Ekko

The large airframe. **Electronics are identical to Don**; the frame and
propellers are what set it apart.

The ROS 2 container on the companion computer is named `uav_ekko` and the OCS
lists the aircraft as `UAV1`, which suggests Ekko is the airframe this repo
actually targets — **please confirm**, because it determines which aircraft the
bringup, geofence and OCS heartbeat in this repo are written against.

**Frame** — 9IMOD IDF-17
*(specifications below are the vendor's listing text and are not independently
verified)*

| | |
|---|---|
| Model | 9IMOD IDF-17 |
| Size | 760 mm |
| Material | 2.0 mm carbon fibre |
| Prop capacity | Up to 17 in |
| Rated max load | 10 kg |
| Landing gear | Tripod legs included |
| Vendor rating | 8S LiPo — **not usable, see ESC ceiling below** |

**Avionics and propulsion** — same as Don

| | |
|---|---|
| Flight controller | Pixhawk Cube Orange+ |
| Navigation | u-blox M9N GNSS with external magnetometer |
| Companion computer | Jetson Orin Nano |
| ESC | Lumenier Elite Pro 60A 2-6S AM32 4-in-1 (fleet-common) |
| Motors | MAD 4014 IPE (fleet-common) |
| Propellers | **16 in — planned, not yet fitted** (pitch TBC) |
| RC link | RadioMaster Pocket → RP3 ELRS, 2.4 GHz |
| Telemetry | 915 MHz FHSS, 100 mW EIRP |

**Propeller clearance checks out.** On a 760 mm quad X, adjacent motors sit
about 537 mm apart. 16 in props are 406 mm across, leaving roughly 131 mm of
tip-to-tip clearance — comfortable, and consistent with the frame being sold as
17 in capable.

**Battery is capped at 6S**, despite the frame being advertised for 8S — the
Lumenier ESC is rated 2–6S and is the binding constraint. A 760 mm airframe on
16 in props will want more capacity than Don's 6000 mAh pack; size it as 6S
with a higher mAh rather than reaching for more cells.

**Still TBC** — firmware version, battery part number, payload fit-out,
flotation, and all masses.

---

## Fitz

Second Pixhawk 1 airframe. The values below are read out of the onboard logs,
so they are confirmed rather than intended.

**Confirmed from dataflash logs `00000064.BIN` / `00000066.BIN`**

| | |
|---|---|
| Firmware | ArduCopter 4.7.0 (`1511f271`), ChibiOS `4f34e217` |
| Board target | `Pixhawk1-1M` — **2 MB flash board running the cut-down 1 MB build** |
| Board ID | `00270023 3137510C 33343537` |
| IO coprocessor | `420 1001 411FC231` |
| Airframe | Quad X |
| ESC output | DShot600 on outputs 9–12; PWM on 1–8 and 13–14 |
| Motor mapping | SERVO9→M3, SERVO10→M2, SERVO11→M1, SERVO12→M4 — **non-standard, do not assume 1:1** |
| RC protocol | SBUS |
| GNSS | u-blox M8 series, 230400 baud (ROM CORE 3.01, 107888) |
| Compasses | 2 — one internal, one external in the GPS module |
| IMUs | 2, fast sampling enabled (8 kHz gyro / 1 kHz accel) |
| Battery monitor | Analog **voltage only** (`BATT_MONITOR=3`), added 29 Aug 2026 |
| Not fitted | Current sensing, rangefinder, airspeed sensor |

**Airframe and propulsion** *(from the team, not the logs)*

| | |
|---|---|
| Frame | Tarot Iron *(650 assumed — TBC)* |
| Companion computer | Jetson Orin Nano |
| ESC | Lumenier Elite Pro 60A 2-6S AM32 4-in-1 (fleet-common) |
| Motors | MAD 4014 IPE (fleet-common) |
| Propellers | 14 in *(pitch TBC — Don runs 5.5)* |
| Battery | 6S LiPo, `BATT_CAPACITY` set to 4200 mAh *(pack part number TBC)* |
| Payload | None fitted |

### Mass and MTOM

| | |
|---|---|
| Empty mass | **TBC** — needs weighing without battery |
| Operating mass, no payload | **2800 g** (measured, flights 1 and 3 on 29 Aug 2026) |
| Heaviest mass flown | **3485 g** (flight 2, with a 1.5 lb suspended load) |
| **MTOM** | **4.6 kg — provisional, to be verified by flight test** |

**How the MTOM was derived.** `MOT_THST_EXPO` linearises throttle against
thrust, so `CTUN.ThO` at hover is the fraction of full-scale thrust being used.
Inverting that at each measured hover point:

| Flight | Mass | Hover `ThO` | Implied full-scale thrust |
|---|---|---|---|
| F1 | 2800 g | 0.281 | 9964 g |
| F3 | 2800 g | 0.304 | 9211 g |
| F2 | 3485 g | 0.368 | 9470 g |

Mean **9548 g** of full-scale thrust with 8 % scatter across the three points;
**8776 g** usable once `MOT_SPIN_MAX = 0.95` is applied. At the standard 2:1
thrust-to-weight design rule that gives **4774 g**.

Two independent lines converge on 4.6 kg, so that is what to declare:

- 4774 g from the thrust measurement above, rounded down for margin
- Don declares 4.6 kg on **identical propulsion** — same MAD 4014 IPE motors,
  same Lumenier ESC, same 14 in props. At 4600 g Fitz would hover at `ThO`
  0.482, a 2.08:1 thrust-to-weight ratio.

> **Caveats.** This is extrapolated from hover, not measured at full throttle —
> the highest throttle in any log is 0.417. It assumes `MOT_THST_EXPO = 0.65`
> (the default) is correct for this motor and prop, which has not been
> verified, and it takes no account of voltage sag at high current since there
> is no current sensor fitted. Treat 4.6 kg exactly as Don does: **provisional,
> pending flight test at mass.**

Demonstrated thrust-to-weight at hover: 3.56:1 at 2800 g, **2.72:1 at 3485 g**.

### Flight test history

See [`0829_flight_test`](0829_flight_test) for the field notes.

| Date | Log | Flights | Notes |
|---|---|---|---|
| 28 Aug 2026 | `00000064.BIN` | 1 | Yaw authority failure — missing arm screws |
| 29 Aug 2026 | `00000066.BIN` | 3 | Post-repair. Loiter, RTL, BRAKE, POSHOLD all exercised |
| 29 Aug 2026 | `00000067.BIN` | 1 | 2800 g + pool noodle, wind. Reached 11.6 m / 3.2 m/s |
| 29 Aug 2026 | `00000068.BIN` | 2 | 3485 g with swinging 1.5 lb load, then 2800 g |

Trend across those logs:

| | log 64 | log 66 | log 67/68 |
|---|---|---|---|
| Yaw mixer bias | +160 µs | −28 µs | −31 to −34 µs |
| Hands-off Loiter creep | — | 0.125 m/s | **0.000 m/s** |
| VibeZ mean @ ThO 0.30 | 11.1 | 8.5 | **16.6–19.6** |
| Accel clipping | 0 | 0 | **42 / 505 / 0** |

**Open regression: vibration.** At matched throttle it has roughly doubled
since log 66, and it scales with rotor RPM — the signature of a rotating
imbalance, not a structural resonance. It is present in the unloaded flights,
so it is not the suspended payload. IMU1 is 4–5× noisier than IMU0 and EKF
core 1 went unhealthy in flight 1 (mag innovation 5.34, position 2.17, all
rejected) while core 0 stayed clean and remained primary throughout.

**Known gaps**

- **No current sensing.** `BATT_MONITOR=3` is voltage-only, so `Curr`,
  `CurrTot` and `EnrgTot` log as NaN and endurance cannot be derived. The
  current-sensor parameters are *already configured* (`BATT_CURR_PIN=3`,
  `BATT_AMP_PERVLT=37.88`, `BATT_CAPACITY=4200`) — setting `BATT_MONITOR=4`
  would turn them on.
- **`ARMING_SKIPCHK = -1`** skips every arming check, which makes the
  configured `BATT_ARM_VOLT = 21.6` inert. Set it to `0`.
- Running the 1 MB firmware build on a 2 MB board.
- Empty mass, frame model, prop pitch and battery part number still **TBC**.

**Repair history** — see [Fleet maintenance notes](#fleet-maintenance-notes).

---

## Common component reference

Shared across the fleet unless a per-aircraft section says otherwise.

| Component | Part | Link |
|---|---|---|
| Frame 1 — Don, Fitz | Tarot IRON MAN 650 Folding Carbon Fiber Quadcopter Frame | — |
| Frame 2 — Ekko | 9IMOD IDF-17, 760 mm, 2.0 mm carbon fibre, 17 in prop capacity, 10 kg rated load, tripod legs | https://www.aliexpress.us/item/3256810008919823.html |
| Transmitter | RadioMaster Pocket | https://radiomasterrc.com/products/pocket-radio-controller-m2 |
| Receiver | RadioMaster RP3 ExpressLRS 2.4 GHz nano | https://radiomasterrc.com/products/rp3-expresslrs-2-4ghz-nano-receiver?variant=46486353674432 |
| Flight controller | Pixhawk (HAWKS WORK, with damper) | https://www.amazon.com/HAWKS-WORK-Pixhawk-Controller-Absorber/dp/B0CTZTJD4J |
| Companion computer | NVIDIA Jetson Orin Nano | https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html |
| ESC — **all three** | Lumenier Elite Pro 60A 2-6S AM32 4-in-1, 30×30 | https://www.lumenier.com/products/lumenier-elite-pro-60a-2-6s-am32-4-in-1-esc-30x30 |
| Camera | SIYI A8 mini | https://www.airbot-systems.com/wp-content/uploads/2025/01/A8-mini-User-Manual-v1.6.pdf |
| Motors — **all three** | MAD Components 4014 IPE | https://www.mad-motor.com/products/madcomponents4014ipe |
| Propellers — Don, Fitz | 14 × 5.5 carbon fibre (2CW + 2CCW) | https://speedyfpv.com/products/4pcs-14x5-5-carbon-fiber-propellers-1455-2cw-2ccw-3k-carbon-fiber-balanced |
| Propellers — Ekko | 16 in, planned — not yet sourced | TBC |
| GNSS | u-blox M8N module | https://www.amazon.com/M8N-GPS-Module-Controller-Receiver/dp/B0FLV6RTY1 |
| Battery | Tattu 6000 mAh 6S1P 35C (Don); others TBC | — |

---

## Fleet maintenance notes

**Motor mount fasteners are a recurring failure mode across the fleet.** Two
aircraft have now lost thrust or control authority to the same class of fault,
both diagnosed from motor-output asymmetry in the logs:

| Aircraft | Event | Root cause | Asymmetry before → after |
|---|---|---|---|
| Don | Motor thrust loss during automatic RTL at ~5.8 m | Motor mounting screws of incorrect length contacting the stator windings, plus a marginal motor signal connector. Both motors replaced, screw lengths corrected, connector strain relief added | 175 µs → 45 µs |
| Fitz | Constant uncommanded yaw torque; yaw integrator saturated, 68° uncommanded yaw on takeoff | Missing arm screws | ~330 µs → 67 µs |

**Add motor-mount fastener inspection (correct length *and* presence) to the
pre-flight checklist on every airframe.**

Other resolved items on Don:

- Uncommanded disarm at ~30 m — a transmitter switch had been assigned to a
  direct disarm function and was activated in flight. Replaced with a dedicated
  arm/disarm assignment.
- Structural resonance in the battery mounting, identified from vibration
  spectra and eliminated by redesigning the mount. Vibration on the affected
  axis fell from 50 m/s² to 6.5 m/s².

---

## Appendix B readiness

Garuda Robotics requires one Appendix B per UAV type **or materially different
configuration**. Status:

| Aircraft | Appendix B | Note |
|---|---|---|
| Don | **Submitted** 27 Aug 2026 | Tattu 6000 config; a second form covers the planned Tattu 16000 |
| Ekko | Not started | Different frame class (760 mm) and prop size — needs its own |
| Fitz | Not started | Different FC, GNSS and battery system — needs its own |

### Fitz — draft answers

Everything below is read from the flight logs unless marked TBC. Fill the TBCs
before submitting.

**B2 Specifications**

| Field | Fitz |
|---|---|
| UAV make / model | Tarot / Tarot Iron *(650 — TBC)* |
| UAV type | Multirotor (quad X) |
| Custom or COTS | Modified COTS |
| Empty mass | **TBC** |
| Operating mass | 2800 g measured; 3485 g demonstrated with suspended load |
| MTOM | 4.6 kg provisional — see [Mass and MTOM](#mass-and-mtom) |
| Dimensions | **TBC** *(0.50 × 0.50 × 0.44 m, 1.02 m diagonal if the same Tarot 650 as Don)* |
| Launch method | Self propelled |
| Landing / recovery | Vertical landing, pilot-commanded or RTL; LAND mode available. RTL demonstrated in `00000067.BIN` |
| Flotation | Closed-cell foam (pool noodle), flown 29 Aug 2026 |
| Propeller size | 14 in *(pitch TBC)* |
| Manual control link | 2.4 GHz ExpressLRS, RadioMaster Pocket → RP3, receiver set to SBUS. **Power output TBC** |
| Telemetry link | **TBC** — `SERIAL1` and `SERIAL2` are both MAVLink2 |
| Video link | None |
| Wi-Fi / other | Via Jetson Orin Nano — **TBC** |
| Flight controller | Pixhawk 1 (2.4.8-class), ArduCopter 4.7.0 |
| Navigation | u-blox M8N GNSS, two magnetometers |
| Obstacle avoidance | None fitted. Software geofence only |

**B4 Battery system**

| Field | Fitz |
|---|---|
| Type / chemistry | LiPo, 6S1P |
| Voltage | 22.2 V nominal, 25.2 V maximum |
| Capacity | 4200 mAh per `BATT_CAPACITY` — **confirm against the actual pack** |
| Number carried | 1 |
| Endurance | **Cannot be derived** — no current sensor, so no consumed-mAh data |
| Arming inhibit | `BATT_ARM_VOLT` 21.6 V — **currently inert**, see open items |
| Low battery | 21.6 V sustained 10 s → RTL |
| Critical battery | 19.8 V → RTL |

Observed: 24.36 → 23.72 V across flight 2 (1.11 V sag), 23.75 → 23.48 V across
flight 3. Never approached either threshold.

**B5 Operations and controls**

| Field | Fitz |
|---|---|
| Flight modes on switch | BRAKE / STABILIZE / STABILIZE / ALT_HOLD / STABILIZE / LOITER |
| RTL | Aux switch, `RC7_OPTION = 4` |
| Emergency stop | Rudder arm/disarm (`ARMING_RUDDER = 2`), `DISARM_DELAY` 10 s |
| Loss of manual control | `FS_THR_ENABLE = 1`, threshold 975 |
| Loss of GCS link | `FS_GCS_ENABLE = 1`, 5 s timeout |
| EKF failsafe | `FS_EKF_ACTION = 1`, threshold 0.8 |
| Crash detection | `FS_CRASH_CHECK = 1` |
| Vibration failsafe | `FS_VIBE_ENABLE = 1` |
| Dead reckoning | `FS_DR_ENABLE = 2`, 30 s timeout |
| Geofence | Enabled — cylindrical, 30 m radius, 30 m ceiling, −10 m floor, 2 m margin, breach action RTL |
| GCS | **TBC** — Don uses QGroundControl over 915 MHz |

**B6 Declared operating limitations**

| Field | Fitz |
|---|---|
| Maximum altitude | 30 m AGL, geofence-enforced |
| Maximum distance | 30 m radius, geofence-enforced |
| Maximum speed | `LOIT_SPEED_MS` 5 m/s; highest observed 3.2 m/s |
| Maximum wind | **TBC** — declare from flight test |
| Maximum flight duration | **TBC** — blocked on current sensing |
| Water operation | Not waterproof; foam flotation only |

**B7 Evidence**

| Field | Fitz |
|---|---|
| Last successful test flight | 29 Aug 2026, verified by onboard log |
| Prior flight test logs | `00000064` / `00000066` / `00000067` / `00000068.BIN` |
| Build / configuration record | This file; parameter exports **TBC** |
| Known repair history | Missing arm screws causing loss of yaw authority — see [Fleet maintenance notes](#fleet-maintenance-notes) |
| Pre-flight checklist | **TBC** — Don has one, Fitz does not |

---

## Open items

- [ ] **Ekko** — confirm whether it is the airframe this repo targets
      (`uav_ekko` container, `UAV1` in the OCS). Remaining unknowns: firmware
      version, battery, payload fit-out, flotation, masses.
- [ ] Size Ekko's battery as **6S** — the frame is advertised for 8S but the
      Lumenier ESC is rated 2–6S and caps it. Go for more capacity, not more
      cells.
- [ ] Confirm the fleet is three aircraft. This file previously said four.
- [x] ~~Fit a battery monitor on Fitz~~ — done 29 Aug 2026, voltage only.
- [ ] Set Fitz `BATT_MONITOR = 4` to enable **current sensing**. The pin and
      scaling parameters are already configured; without it there is no
      consumed-mAh data and endurance cannot be declared in Appendix B.
- [ ] Set Fitz `ARMING_SKIPCHK = 0`. All arming checks are skipped, which makes
      the configured `BATT_ARM_VOLT = 21.6 V` inert.
- [ ] Calibrate Fitz `BATT_VOLT_MULT` against a voltmeter — it is at the
      generic default of 11.0, and the voltage failsafes are only as good as it.
- [ ] **Resolve Fitz's vibration regression** before further mass testing.
      Doubled at matched throttle since log 66 and scales with rotor RPM.
      Enable `INS_LOG_BAT_MASK` for a spectrum, then check props, motor
      bearings and the FC damper mount.
- [ ] Investigate Fitz **IMU1 / EKF core 1** — 4–5× the accel noise of IMU0,
      and core 1 rejected mag, velocity and position in flight 1. Core 0 is
      healthy and primary, so there is currently no redundancy.
- [ ] Verify what Fitz's `RC6_OPTION = 154` is assigned to. Don's crash history
      includes an uncommanded disarm from a switch assigned to a direct disarm
      function — confirm this is not the same hazard.
- [ ] Weigh Fitz **empty** and measure its dimensions for Appendix B.
- [ ] Write a **pre-flight checklist for Fitz** (Don has one).
- [ ] Verify Fitz's `MOT_THST_EXPO` — the MTOM figure assumes the default 0.65
      is correct for the MAD 4014 / 14 in combination.
- [ ] Move Fitz to the full **Pixhawk1 (2 MB)** firmware build.
- [ ] Confirm Fitz's frame model, propeller pitch and battery.
- [ ] Source Ekko's 16 in props, then **tune Ekko from scratch** — same motors
      and ESCs as the other two, but ~30 % more disc area, so hover throttle,
      thrust expo and rate PIDs will not carry over.
- [ ] Check 16 in prop-tip clearance against Ekko's frame before first spin-up.
- [ ] **Appendix B forms for Ekko and Fitz.** GR requires one per UAV type or
      materially different configuration; only Don's is submitted.
- [ ] Retune Don's telemetry radio to **920–925 MHz** for IMDA compliance before
      arrival in Singapore.
- [ ] Decide and record whether Wi-Fi is enabled in flight (open TBC in Don's
      Appendix B).
- [ ] Declare a maximum wind speed limit from flight test (Don's Appendix B
      suggests an interim 6 m/s sustained / 8 m/s gust, pending measurement).
- [ ] Record battery part numbers for Ekko and Fitz.
