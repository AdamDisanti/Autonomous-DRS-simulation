# Paste this into Claude Code at repository setup

Read `CLAUDE.md` in full, then read `docs/Active_Aero_Software_Context.pdf` and inspect the COPoP diagram and system diagrams in `docs/diagrams/`.

We are building the UCF Formula SAE active aero **supervisory state machine and offline simulation/fault-detection framework**. Start with **Phase 1**. Before changing any code:

1. Summarize the controller states, parent/child hierarchy, transition guards, and proposed strict priorities in a table.
2. Identify unsafe or ambiguous assumptions (manual hardware override, true physical failure to HDF, jammed output, stale CAN values, recovery from lost supply). Explicitly separate *commanded aero target* from *measured/verified element position*.
3. Propose lightweight Python architecture, data contracts, and first tests. Maintain timing deterministically using injected monotonic clock values, and keep all actual CAN IDs and production thresholds as TBD.
4. Then implement the smallest useful Python package with pytest tests for boot/wait, valid inputs/HDF, LD eligibility, braking/cornering precedence, manual inhibit, a fault latch, and incomplete physical return.
5. Give runnable commands, test results, and the next implementation step. **Do not write real actuator commands, assume any physical fail-safe behavior, or claim this is vehicle-ready.**

After we approve the state table, develop a three-channel actuator plant, fault injectors, time-series trace viewer, and later portability toward ESP32-S3/ESP-IDF firmware.
