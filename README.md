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

## Getting started

> **TL;DR:** install Python 3.11+ from [python.org](https://www.python.org/downloads/), get the code, then
> double-click **`start-windows.bat`** (Windows) or **`start-mac.command`** (Mac). The first run sets
> everything up in 1–2 minutes and opens the dashboard in your browser.

Python 3.11+ is the only thing you install yourself. The C compiler and every other package are downloaded
automatically into the project folder the first time you run it. Nothing is installed system-wide.

### Windows

1. **Install Python.** Download the latest Python 3 from
   [python.org/downloads](https://www.python.org/downloads/) and run the installer with its default options.
   This also installs the `py` command the project uses.
2. **Get the code.** Either:
   - with [Git for Windows](https://git-scm.com/download/win): `git clone https://github.com/AdamDisanti/Autonomous-DRS-simulation.git`, or
   - on GitHub, *Code → Download ZIP*, then right-click the ZIP → *Extract All*.
3. **Start the simulator.** Open the project folder in File Explorer and double-click **`start-windows.bat`**.
   If Windows shows "Windows protected your PC" (common after a ZIP download), click *More info → Run anyway*.
4. **Wait for the first-time setup.** A black window shows progress. The first run takes 1–2 minutes and needs
   internet. When it's done, the dashboard opens in your browser at `http://127.0.0.1:8050`.
5. **Keep the black window open** while you use the dashboard. Close it (or press Ctrl+C) to stop the simulator.

Next time, just double-click `start-windows.bat` again; it starts in a few seconds.

To use a terminal instead: in File Explorer, click the folder's address bar, type `powershell`, and press Enter.
Then run `py aero.py` (or any command listed below, with `py` in place of `python`).

### Mac

1. **Install Python.** Download the latest Python 3 from [python.org/downloads](https://www.python.org/downloads/)
   and run the installer. Don't rely on the `python3` that comes with Xcode; it is too old.
2. **Get the code:** `git clone https://github.com/AdamDisanti/Autonomous-DRS-simulation.git` in Terminal, or on GitHub use *Code → Download ZIP* and unzip it.
3. **Start the simulator:** double-click **`start-mac.command`**. The first time, macOS may block it: right-click
   it → *Open* → *Open*. Or, in Terminal inside the project folder, run `python3 aero.py`.
4. The first run takes 1–2 minutes and needs internet, then the dashboard opens in your browser. Keep the
   Terminal window open while you use it; close it or press Ctrl+C to stop.

### Getting updates

After pulling new code (`git pull`) or downloading a new ZIP, start the simulator the same way. `aero.py`
notices changed C code or package versions and rebuilds or reinstalls on its own.

### Other commands

Run these in a terminal in the project folder (use `py` on Windows and `python3` on Mac in place of `python`):

```bash
python aero.py setup        # reinstall/rebuild and run every test; do this if something seems broken
python aero.py test         # C smoke test + pytest suite
python aero.py scenarios    # run every scripted scenario -> results/<id>/{trace.csv,result.json,timeline.png}
python aero.py clean        # delete .venv, build and results; the next run starts from scratch
python aero.py dashboard --port 8060 --no-browser   # pick a port / don't open a browser tab
```

### If something goes wrong

- *"Python was not found"*, *"'py' is not recognized"* or *"needs Python 3.11 or newer"*: install Python from
  python.org (step 1), then open a **new** terminal window or double-click the launcher again.
- *The black window flashes and closes, or shows "Press any key"*: read the message just above that prompt. It
  usually says what to do.
- *"aero_core.dll is in use"*: a simulator is already running. Close its window first.
- *Pip install fails*: check your internet connection (some campus or guest Wi-Fi networks block downloads),
  then run setup again.
- *Mac says the .command file "cannot be opened"*: right-click it → Open → Open. If the file isn't executable
  after a ZIP download, run `python3 aero.py` from Terminal instead.
- *Anything else*: run `python aero.py clean`, then `python aero.py setup`, and send the output to the software lead.

### Dashboard

The dashboard is a pit-wall style page with two modes:

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

### How the build works

`aero.py` compiles `core/src/aero_core.c` with [zig](https://ziglang.org/)'s C compiler, which pip installs
from the `ziglang` package into `.venv`. That gives everyone the same compiler version on every OS, with no
Xcode, Visual Studio, MSYS2, CMake or make. Flags are `-std=c99 -O2 -Wall -Wextra -Wpedantic -Wshadow
-Werror`, so any warning fails the build. Output goes to `build/aero_core.{dll,dylib,so}` (loaded by the
simulator through ctypes) and `build/test_core_c` (the plain-C smoke test). The ESP32 build will use ESP-IDF's
own build system (Phase 6).

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
aero.py                           setup / build / test / run entry point (all platforms)
start-windows.bat, start-mac.command   double-click launchers for aero.py
requirements.txt                  pinned Python packages, incl. the zig C compiler
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
