"""Core invariants checked over every simulated trace.

These are properties that must hold at every tick no matter the scenario.
They check the supervisor's *logic*; they are not evidence of physical safety.
"""
from __future__ import annotations

import math

from .core import CHANNELS, SIGNALS


def _stability_demand(r: dict, cfg: dict) -> bool:
    return (r["brake"] > cfg["hdf_brake_bar"]
            or abs(r["steering"]) > cfg["hdf_steering_rad"]
            or abs(r["yaw_rate"]) > cfg["hdf_yaw_rate_rps"]
            or abs(r["lat_accel"]) > cfg["hdf_lat_accel_mps2"])


def check(trace: list[dict], cfg: dict) -> list[str]:
    violations: list[str] = []
    n_ch = int(cfg["channels"]["count"])
    chans = CHANNELS[:n_ch]
    tol = cfg["position_tolerance_deg"]
    allowed_cmds = [set(), set()]
    for i in range(n_ch):
        allowed_cmds[0].add(cfg["channels"]["hdf_position_deg"][i])
        allowed_cmds[1].add(cfg["channels"]["ld_position_deg"][i])
    required = [s for s in SIGNALS if cfg["signals"][s].get("required", True)]

    def bad(r, msg):
        violations.append(f"t={r['t_ms']}ms: {msg}")

    prev = None
    for r in trace:
        # A dead controller makes no decisions; its last output is stale by
        # definition. Only the physical (no fabricated motion) check applies.
        if r["state"] == "REDUCED_DRAG" and not r["controller_dead"]:
            if r["manual_inhibit"]:
                bad(r, "REDUCED_DRAG while manual inhibit active")
            if r["latched_faults"] or r["active_faults"]:
                bad(r, f"REDUCED_DRAG with faults {r['active_faults'] or r['latched_faults']}")
            invalid = [s for s in required if r[f"{s}_status"] != "FRESH_VALID"]
            if invalid:
                bad(r, f"REDUCED_DRAG with invalid inputs {invalid}")
            elif _stability_demand(r, cfg):
                bad(r, "REDUCED_DRAG during braking/cornering/stability demand")
        if r["state"] == "INTERMEDIATE_VARIABLE_AERO" and not cfg.get("variable_aero_enabled"):
            bad(r, "entered variable aero while feature disabled")
        if not r["actuator_power"] and r["command_enabled"]:
            bad(r, "command enabled with actuator power cut")

        all_confirmed = True
        for ch in chans:
            status, cmd, sensor = r[f"{ch}_status"], r[f"{ch}_cmd"], r[f"{ch}_sensor"]
            if status == "CONFIRMED":
                if math.isnan(sensor) or abs(sensor - cmd) > tol:
                    bad(r, f"{ch} CONFIRMED without independent sensor evidence")
            else:
                all_confirmed = False
            if not math.isnan(cmd) and not any(math.isclose(cmd, a, abs_tol=1e-4) for s in allowed_cmds for a in s):
                bad(r, f"{ch} command {cmd} outside configured HDF/LD positions")
        if r["output_confirmed"] and not all_confirmed:
            bad(r, "output_confirmed while a channel is not CONFIRMED")

        if prev is not None:
            if r["state"] != prev["state"] and r["reason"] == "NONE":
                bad(r, f"transition {prev['state']}->{r['state']} without a reason")
            for ch in chans:
                moved = not math.isclose(r[f"{ch}_true_pos"], prev[f"{ch}_true_pos"], abs_tol=1e-9)
                if moved and (not r["command_enabled"] or not r["actuator_power"]):
                    bad(r, f"{ch} moved with no command/power (fabricated motion)")
        prev = r
    return violations
