"""Scripted scenarios: vehicle keyframes + timed fault injections + expectations.

Scenario file (JSON):
{
  "scenario_id": "...", "description": "...",
  "config": "simulation_example_only.json",   # file in config/
  "duration_ms": 3000, "tick_ms": 5, "seed": 0, "service_mode": false,
  "vehicle": [{"t_ms": 0, "speed": 0, "throttle": 0, ...}, ...],   # linear interpolation
  "profile": "autocross", "profile_start_ms": 1000,  # OR: synthetic lap from track.py
  "events": [{"type": "manual_inhibit", "t_ms": 1000, "duration_ms": 500}, ...],
  "expect": [{"t_ms": 900, "state": "REDUCED_DRAG"},
             {"from_ms": 0, "to_ms": 3000, "never_state": "REDUCED_DRAG"}]
}

Event types (duration_ms omitted = stays active until the end):
  manual_inhibit, power_cut, can_silence, controller_dead,
  signal_stale {signal}, signal_source_invalid {signal}, signal_override {signal, value (null = NaN)},
  noise {signal, std}, jam {channel}, sensor_fail {channel}, comms_loss {channel},
  branch_power_loss {channel}, fault_reset (one-tick pulse), service_exit (one-tick pulse)
"""
from __future__ import annotations

import bisect
import json
import math
from pathlib import Path

from .core import SIGNALS
from .invariants import check as check_invariants
from .sim import Sim
from .track import build_lap

ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = ROOT / "config"
SCENARIO_DIR = ROOT / "sim" / "scenarios"

PULSES = {"fault_reset", "service_exit"}


def load_config(name: str) -> dict:
    return json.loads((CONFIG_DIR / name).read_text())


def load_scenario(path: str | Path) -> dict:
    return json.loads(Path(path).read_text())


def _interp(keyframes: list[dict], signal: str, t: int) -> float | None:
    pts = [(k["t_ms"], k[signal]) for k in keyframes if signal in k]
    if not pts:
        return None
    times = [p[0] for p in pts]
    i = bisect.bisect_right(times, t)
    if i == 0:
        return pts[0][1]
    if i == len(pts):
        return pts[-1][1]
    (t0, v0), (t1, v1) = pts[i - 1], pts[i]
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0)


def _reset_injections(sim: Sim) -> None:
    sim.manual_inhibit = False
    sim.actuator_power = True
    sim.can_silent = False
    sim.controller_dead = False
    sim.stale.clear()
    sim.source_invalid.clear()
    sim.override.clear()
    sim.noise_std.clear()
    for p in sim.plants:
        p.jammed = p.sensor_failed = p.comms_lost = p.supply_lost = False


def _apply(sim: Sim, ev: dict) -> None:
    kind = ev["type"]
    if kind == "manual_inhibit":
        sim.manual_inhibit = True
    elif kind == "power_cut":
        sim.actuator_power = False
    elif kind == "can_silence":
        sim.can_silent = True
    elif kind == "controller_dead":
        sim.controller_dead = True
    elif kind == "signal_stale":
        sim.stale.add(ev["signal"])
    elif kind == "signal_source_invalid":
        sim.source_invalid.add(ev["signal"])
    elif kind == "signal_override":
        sim.override[ev["signal"]] = math.nan if ev.get("value") is None else float(ev["value"])
    elif kind == "noise":
        sim.noise_std[ev["signal"]] = float(ev["std"])
    elif kind == "jam":
        sim.plant(ev["channel"]).jammed = True
    elif kind == "sensor_fail":
        sim.plant(ev["channel"]).sensor_failed = True
    elif kind == "comms_loss":
        sim.plant(ev["channel"]).comms_lost = True
    elif kind == "branch_power_loss":
        sim.plant(ev["channel"]).supply_lost = True
    elif kind == "fault_reset":
        sim.fault_reset = True
    elif kind == "service_exit":
        sim.service_exit = True
    else:
        raise ValueError(f"unknown event type {kind!r}")


def run(sc: dict) -> tuple[Sim, dict]:
    config = load_config(sc.get("config", "simulation_example_only.json"))
    tick = int(sc.get("tick_ms", 5))
    sim = Sim(config, tick_ms=tick, seed=int(sc.get("seed", 0)), service_mode=bool(sc.get("service_mode")))
    keyframes = sorted(sc.get("vehicle", []), key=lambda k: k["t_ms"])
    events = sc.get("events", [])
    lap = build_lap() if sc.get("profile") == "autocross" else None
    lap_start = int(sc.get("profile_start_ms", 0))

    for _ in range(int(sc["duration_ms"]) // tick):
        t = sim.now + tick
        if lap is not None:
            sim.vehicle.update(lap.signals_at(max(0, t - lap_start) / 1000.0))
        for name in SIGNALS:
            v = _interp(keyframes, name, t)
            if v is not None:
                sim.vehicle[name] = float(v)
        _reset_injections(sim)
        for ev in events:
            start = int(ev["t_ms"])
            if ev["type"] in PULSES:
                if start <= t < start + tick:
                    _apply(sim, ev)
            elif start <= t and ("duration_ms" not in ev or t < start + int(ev["duration_ms"])):
                _apply(sim, ev)
        sim.step()

    return sim, evaluate(sc, sim.trace, config)


def transitions(trace: list[dict]) -> list[dict]:
    out, prev = [], None
    for r in trace:
        if prev is None or r["state"] != prev:
            out.append({"t_ms": r["t_ms"], "state": r["state"], "reason": r["reason"]})
        prev = r["state"]
    return out


def evaluate(sc: dict, trace: list[dict], config: dict) -> dict:
    failures: list[str] = []
    times = [r["t_ms"] for r in trace]

    def row_at(t):
        i = bisect.bisect_right(times, t) - 1
        return trace[max(i, 0)]

    for exp in sc.get("expect", []):
        if "t_ms" in exp:
            r = row_at(exp["t_ms"])
            for key in ("state", "target", "reason", "output_confirmed"):
                if key in exp and r[key] != exp[key]:
                    failures.append(f"t={exp['t_ms']}: expected {key}={exp[key]}, got {r[key]}")
            for ch, st in exp.get("channel_status", {}).items():
                if r[f"{ch}_status"] != st:
                    failures.append(f"t={exp['t_ms']}: expected {ch}={st}, got {r[f'{ch}_status']}")
            for f in exp.get("latched_faults_include", []):
                if f not in r["latched_faults"].split("|"):
                    failures.append(f"t={exp['t_ms']}: expected latched fault {f}, got '{r['latched_faults']}'")
            for flag in exp.get("inhibit_include", []):
                if flag not in r["inhibit"].split("|"):
                    failures.append(f"t={exp['t_ms']}: expected inhibit {flag}, got '{r['inhibit']}'")
        else:
            window = [r for r in trace if exp["from_ms"] <= r["t_ms"] <= exp["to_ms"]]
            if "never_state" in exp:
                hits = [r["t_ms"] for r in window if r["state"] == exp["never_state"]]
                if hits:
                    failures.append(f"{exp['never_state']} occurred at t={hits[0]} (window {exp['from_ms']}-{exp['to_ms']})")
            for ch, st in exp.get("never_channel_status", {}).items():
                hits = [r["t_ms"] for r in window if r[f"{ch}_status"] == st]
                if hits:
                    failures.append(f"{ch}={st} occurred at t={hits[0]}")
            if "max_transitions" in exp and window:
                n = window[-1]["transition_count"] - window[0]["transition_count"]
                if n > exp["max_transitions"]:
                    failures.append(f"{n} transitions in window, max {exp['max_transitions']} (chatter)")

    violations = check_invariants(trace, config)
    return {
        "scenario_id": sc["scenario_id"],
        "description": sc.get("description", ""),
        "passed": not failures and not violations,
        "expectation_failures": failures,
        "invariant_violations": violations[:50],
        "transitions": transitions(trace),
    }
