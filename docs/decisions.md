# Decisions log

Records what has been decided, who decided it, and what is still open.
Authority order follows `CLAUDE.md`: team/advisor decisions > COPoP chart > M2 EDS > proposals.

## Decided — 2026-10-28 (software lead, via Claude Code session)

| # | Topic | Decision |
|---|---|---|
| D1 | Purpose | Preliminary placeholder software + state-switching simulation, to understand the implementation before the car. Not vehicle code yet. |
| D2 | Architecture | ESP32 handles low-level control. ESP32 ↔ MoTeC M150 over CAN **in both directions**: reads vehicle data, publishes state/faults/positions for logging. |
| D3 | Language | Supervisor core in **portable C99** (`core/`), with no ESP-IDF calls, no malloc and injected time. Python drives it for simulation through ctypes. |
| D4 | State model | Code states mirror the **COPoP flowchart one-to-one** (8 states). |
| D5 | Sim UI | Scripted scenarios + plots **and** a live local web dashboard. |
| D6 | INHIBITED output | Block LD, **request HDF**, report confirmed / unconfirmed from the output sensors. |
| D7 | Inhibit exit | Releasing the switch (or inputs recovering) → **WAIT_FOR_VALID_INPUTS**, never straight to HDF/LD. |
| D8 | Driver switch model | Software digital input. Separately, a **sim-only actuator power-cut** toggle models a hard power kill. |
| D9 | Fallback recovery | **Split by fault type.** CAN/input faults auto-recover after a stable window. Actuator/sensor/mechanical faults latch until a deliberate reset, and the reset only clears faults whose condition is gone. |
| D10 | Missing CAN data | **One signal stale/invalid → INHIBITED. Whole bus silent → COMMANDED_FALLBACK** (recoverable). |
| D11 | Actuator power cut | → **COMMANDED_FALLBACK**, latched. Position reported UNCONFIRMED/UNKNOWN; passive return to HDF is never assumed. |
| D12 | Channels | 3 (FRONT_LEFT, FRONT_RIGHT, REAR_MAIN), configurable to 4. |
| D13 | Thresholds | Real config (`config/controls_thresholds.json`) is all TBD, and the core refuses to leave POWER_ON_INIT with it. Demo values live only in `config/simulation_example_only.json`. |

## Placeholder behavior Claude chose — please confirm or change

These were needed to make the simulation run. They are **not** team decisions yet.

| # | Placeholder | Where |
|---|---|---|
| P1 | WAIT_FOR_VALID_INPUTS and SERVICE command **nothing** (`target = NONE`, outputs not driven). After an inhibit release the wings are not actively held during WAIT. Should WAIT hold HDF instead? | `aero_core.c: target_for_state` |
| P2 | Service mode has no motion commands at all (calibration moves TBD). Service can only be selected on the first tick after power-on. | `aero_core.c` |
| P3 | Manual inhibit outranks latched faults (state shows INHIBITED; the latched fault reappears as FALLBACK on release). Actuator power loss outranks manual inhibit. | `decide_running` |
| P4 | One channel failing (e.g. FRONT_LEFT comms) → fallback still commands the **healthy** channels to HDF. This can leave a left/right asymmetry. **Open decision #3 in CLAUDE.md.** | `test_one_front_channel_comms_loss_latches` |
| P5 | The fault reset input source is unspecified (driver button? MoTeC display? power cycle?). | `aero_inputs_t.fault_reset_request` |
| P6 | Motion monitoring only checks "did the output sensor reach the command within `motion_timeout_ms`". Motor-vs-output disagreement, current/stall detection and end switches come in Phase 4. | `aero_tick` step 6 |
| P7 | Only 6 vehicle signals (speed, throttle, brake, steering, yaw rate, lateral accel). Wheel speeds and the cross-checks between them aren't modeled yet. | `aero_signal_id_t` |
| P8 | A recovered latched fault still waits `fault_recovery_stable_ms`, then goes through WAIT (`inputs_stable_ms`) before HDF. | `decide_running` |

## Still open (from CLAUDE.md §13, unchanged)

Driver override wiring; passive HDF on power loss; mixed/asymmetric states; actuator models/protocols;
output sensor placement and calibration; real CAN DBC and update rates; all thresholds; fault severity
classes; independent watchdog; power distribution details; competition rule sign-off.
