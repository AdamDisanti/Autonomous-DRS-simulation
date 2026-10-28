# UCF Knights Racing — Active Aero Supervisor (preliminary)

> **SIMULATION ONLY.** This is placeholder software for understanding the supervisory state machine
> before vehicle integration. Thresholds and actuator physics are synthetic. Nothing here is validated,
> fail-safe, or vehicle-ready. An HDF *command* is never treated as proof of an HDF *position*.

The supervisor decides when the aero elements should be in **high downforce (HDF)** or **reduced drag (LD)**.
It also decides how to react when the driver inhibits the system, inputs go bad, or an actuator fails.
The decision logic is a **portable C core** meant to move onto the ESP32 later. Python drives that
same C code with simulated vehicle signals and simulated actuators.

```
 vehicle signals ─▶ ┌──────────────────────┐ ─▶ commands ─▶ actuator plant (sim)
 (MoTeC over CAN)   │  C core (aero_core)  │                    │
 driver switch ───▶ │  validate → faults → │ ◀── output-sensor feedback
 power / comms ───▶ │  priority → state    │ ─▶ state/faults/positions ─▶ MoTeC logging (Phase 2)
                    └──────────────────────┘
```

## Quick start

```bash
make setup        # one time: create .venv with pytest + matplotlib
make all          # build the C core, run the C smoke test and the pytest suite
make scenarios    # run every scripted scenario -> results/<id>/{trace.csv,result.json,timeline.png}
make dashboard    # live web dashboard at http://127.0.0.1:8050
```

### Dashboard

`make dashboard` opens a pit-wall style page with two modes:

- **Manual drive:** you set speed, throttle, brake and steering with sliders or presets.
- **Autocross run:** a synthetic ~50 s lap (`sim/active_aero_sim/track.py`, arbitrary numbers) drives the
  inputs. The track map colours itself by supervisor state as the car goes round, and a summary appears at the
  finish. Use pause (space bar) and 0.5×–8× speed.

In both modes you can inject faults at any time: inhibit switch (`i` key), power cut, CAN silence, stale or
corrupt signals, ESP32 crash, and per-actuator jam / sensor / comms / branch-power faults. Panels:
- **Car side view:** component placement and a wiring overlay that reacts to faults.
- **Supervisor state:** shown with a plain-English reason, plus a mini COPoP chart.
- **Signal flow:** one 5 ms tick.
- **Actuator gauges:** command vs measured position for each wing.
- **Telemetry chart** and **state-change log**.

You need Python ≥ 3.11, CMake and a C compiler. On macOS, the Makefile points CMake at the
Xcode SDK, which works around a linker/SDK mismatch on this machine.

## States (mirrors the COPoP flowchart)

| State | Commands | Leaves when |
|---|---|---|
| POWER_ON_INIT | nothing | config complete → WAIT (or SERVICE if selected at power-on). A TBD config never leaves. |
| SERVICE_CALIBRATION | nothing (service moves TBD) | service exit → WAIT |
| WAIT_FOR_VALID_INPUTS | nothing | all required signals fresh + valid for `inputs_stable_ms` → HDF |
| HIGH_DOWNFORCE | HDF | LD conditions held for `ld_entry_debounce_ms` → LD |
| REDUCED_DRAG | LD | conditions drop below exit thresholds, or brake/steer/yaw/lat demand → HDF **on the same tick** |
| INTERMEDIATE_VARIABLE_AERO | — | unreachable (feature disabled; a config enabling it is rejected) |
| INHIBITED | HDF, then reports confirmed/unconfirmed | switch released or inputs recovered → WAIT |
| COMMANDED_FALLBACK | HDF if actuator power is present | CAN fault: auto-recovers after `fault_recovery_stable_ms`. Latched fault: deliberate reset, and only once the condition is gone. |

**Priority each tick (first match wins), applied from WAIT, HDF, LD, INHIBITED and FALLBACK:**

1. Actuator power lost → FALLBACK
2. Manual inhibit → INHIBITED
3. Latched fault → FALLBACK
4. Whole CAN bus silent → FALLBACK
5. Required signal stale/invalid → INHIBITED (WAIT just keeps waiting)
6. Braking / steering / yaw / lateral demand → HDF
7. LD eligible + debounced → LD
8. Otherwise → HDF

**Position status per channel** comes only from the independent output sensor:
`REQUESTED → IN_MOTION → CONFIRMED`. Otherwise it's `UNCONFIRMED` (timeout, or no power to move) or
`UNKNOWN` (sensor failed). A channel that stays `UNCONFIRMED` while being driven latches `MOTION_TIMEOUT`.

See [docs/decisions.md](docs/decisions.md) for what is decided, what is a placeholder, and what is open.

## Layout

```
CLAUDE.md                         project context (read first)
core/                             portable C99, intended for the ESP32
  include/aero_core.h             types, config, inputs/outputs, API
  src/aero_core.c                 the state machine
  tests/test_core.c               plain-C smoke test (no Python needed)
config/
  controls_thresholds.json        REAL config: all TBD (core refuses to run)
  simulation_example_only.json    SYNTHETIC values for demos/tests only
sim/active_aero_sim/              Python simulation (all decisions come from the C core)
  core.py                         ctypes binding (layout checked against C at load)
  sim.py                          Sim: vehicle signals + C core + plants, one tick at a time
  plant.py                        synthetic rate-limited actuator; never invents motion
  scenario.py                     JSON scenario runner + expectation checks
  invariants.py                   safety-logic properties checked on every trace
  plot.py                         timeline PNGs
  track.py                        synthetic ~50 s autocross lap (arbitrary numbers)
sim/scenarios/                    9 scripted scenarios (lap, inhibit, CAN, power cut, jam, autocross, ...)
sim/dashboard/                    live web dashboard (stdlib server + one HTML page)
tests/                            pytest suite (drives the C core)
scripts/run_scenario.py           CLI scenario runner
docs/                             context PDF, decisions log
```

## Writing a scenario

Copy a file in `sim/scenarios/`. Vehicle keyframes are linearly interpolated per signal, so put a
keyframe wherever a signal should hold a value. Fault events: `manual_inhibit`, `power_cut`,
`can_silence`, `controller_dead`, `signal_stale`, `signal_source_invalid`, `signal_override`, `noise`,
`jam`, `sensor_fail`, `comms_loss`, `branch_power_loss`, `fault_reset`, `service_exit`. Expectations can
check the state/reason/target at a time, or assert that a state never occurs inside a window.
See the docstring in [scenario.py](sim/active_aero_sim/scenario.py).

## Path to the ESP32

`aero_core.c/.h` uses no OS calls, no allocation, no stdio and no wall clock. On the ESP32, a FreeRTOS task
would, every tick:

1. fill `aero_inputs_t` from TWAI CAN frames (MoTeC), the inhibit GPIO, supply sensing and actuator feedback
2. call `aero_tick(&sup, &in, esp_timer_get_time() / 1000)`
3. send `channel_command_deg[]` to the actuator adapters only if `command_enabled`
4. pack the state, reason, faults and positions into CAN frames for MoTeC logging

Steps 1, 3 and 4 are the Phase 2/6 adapters. The core itself should compile unchanged with ESP-IDF.

## Roadmap

- **Phase 2:** mock CAN layer (placeholder IDs and frame packing for MoTeC → ESP32 inputs and ESP32 → MoTeC status), per-signal update rates, wheel speeds.
- **Phase 3–4:** richer plant (motor encoder vs output sensor, current, end switches) and fault monitors (linkage disconnect, stall/jam via current, frozen sensor, asymmetry policy).
- **Phase 5:** CSV log playback, regression reports.
- **Phase 6:** ESP-IDF project wrapping the same core, watchdog, measured loop timing.
