"""Live interactive dashboard for the active aero supervisor simulation.

    python sim/dashboard/server.py            # then open http://127.0.0.1:8050

Two modes:
  manual  - you drive the inputs with sliders / presets
  lap     - a synthetic ~50 s autocross lap (track.py) drives the inputs; you can
            still inject faults mid-lap

A background thread runs the Sim (C core + synthetic plant) at 1x-8x real time.
The browser polls /api/state and posts to /api/control, /api/mode, /api/playback.
Standard library only. SIMULATION ONLY - arbitrary numbers.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from active_aero_sim.core import CHANNELS, SIGNALS  # noqa: E402
from active_aero_sim.scenario import load_config  # noqa: E402
from active_aero_sim.sim import Sim  # noqa: E402
from active_aero_sim.track import build_lap  # noqa: E402

HISTORY_MS = 12_000
HISTORY_EVERY_MS = 20
PLANT_FAULTS = ("jammed", "sensor_failed", "comms_lost", "supply_lost")
TRACK_STRIDE = 4          # track.py points are 0.5 m apart -> send every 2 m
GRID_MS = 1500            # time on the grid before the lap starts (boot + WAIT)
SPEEDS = (0.5, 1, 2, 4, 8)
STOPPED = {name: 0.0 for name in SIGNALS}


def _clean(v):
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return None
    if isinstance(v, float):
        return round(v, 4)
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    return v


class LiveSim:
    def __init__(self, config_name: str):
        self.config_name = config_name
        self.lock = threading.Lock()
        self.lap = build_lap()
        pts = range(0, len(self.lap.x), TRACK_STRIDE)
        self.track = {
            "points": [[round(self.lap.x[i], 2), round(self.lap.y[i], 2)] for i in pts],
            "labels": [self.lap.label[i] for i in pts],
            "duration_s": round(self.lap.duration_s, 2),
            "length_m": round(self.lap.length_m, 1),
        }
        self.speed = 1.0
        self.paused = False
        self.restart(mode="manual")

    # ------------------------------------------------------------ lifecycle
    def restart(self, mode: str = "manual", service_mode: bool = False) -> None:
        with self.lock:
            self.mode = mode if mode in ("manual", "lap") else "manual"
            self.config = load_config(self.config_name)
            self.sim = Sim(self.config, tick_ms=5, service_mode=service_mode)
            self.sim.set_vehicle(**STOPPED)
            self.history: deque[dict] = deque(maxlen=HISTORY_MS // HISTORY_EVERY_MS)
            self.transitions: deque[dict] = deque(maxlen=60)
            self._last_state = None
            self._last_wall = time.monotonic()
            self._budget_ms = 0.0
            self.paused = False
            self.lap_phase = "grid" if self.mode == "lap" else "off"
            self.lap_idx = 0
            self.trail: list[str | None] = [None] * len(self.track["points"])
            self.stats = {"ld_ms": 0, "lap_ms": 0, "ld_entries": 0, "fallbacks": 0, "inhibits": 0}

    # ------------------------------------------------------------ stepping
    def _drive_lap(self, t_next: int) -> None:
        lap_t = (t_next - GRID_MS) / 1000.0
        if lap_t < 0:
            self.lap_phase = "grid"
            self.sim.set_vehicle(**STOPPED)
        elif lap_t <= self.lap.duration_s:
            self.lap_phase = "running"
            self.sim.vehicle.update(self.lap.signals_at(lap_t))
            self.lap_idx = self.lap.index_at(lap_t)
        else:
            self.lap_phase = "finished"
            self.sim.set_vehicle(**STOPPED)
            self.lap_idx = len(self.lap.t) - 1

    def _record(self, r: dict) -> None:
        state = r["state"]
        if state != self._last_state:
            self.transitions.append({"t_ms": r["t_ms"], "state": state, "reason": r["reason"]})
            if self.lap_phase == "running":
                key = {"REDUCED_DRAG": "ld_entries", "COMMANDED_FALLBACK": "fallbacks", "INHIBITED": "inhibits"}.get(state)
                if key:
                    self.stats[key] += 1
            self._last_state = state
        if self.lap_phase == "running":
            self.stats["lap_ms"] += self.sim.tick_ms
            if state == "REDUCED_DRAG":
                self.stats["ld_ms"] += self.sim.tick_ms
            self.trail[self.lap_idx // TRACK_STRIDE] = state
        if r["t_ms"] % HISTORY_EVERY_MS == 0:
            n = self.sim.n_ch
            self.history.append({
                "t": r["t_ms"], "state": r["state"],
                "speed": r["speed"], "throttle": r["throttle"], "brake": r["brake"],
                "pos": [r[f"{c}_true_pos"] for c in CHANNELS[:n]],
                "cmd": [r[f"{c}_cmd"] for c in CHANNELS[:n]],
            })

    def advance(self) -> None:
        with self.lock:
            now = time.monotonic()
            wall_dt, self._last_wall = now - self._last_wall, now
            if self.paused:
                return
            self._budget_ms = min(self._budget_ms + wall_dt * 1000 * self.speed, 2000)
            tick = self.sim.tick_ms
            while self._budget_ms >= tick:
                if self.mode == "lap":
                    self._drive_lap(self.sim.now + tick)
                self.sim.step()
                self._record(self.sim.trace[-1])
                self._budget_ms -= tick
            self.sim.trace.clear()  # live mode keeps only the rolling history

    # ------------------------------------------------------------ controls
    def control(self, msg: dict) -> None:
        with self.lock:
            s = self.sim
            if "vehicle" in msg and self.mode == "manual":
                s.set_vehicle(**{k: float(v) for k, v in msg["vehicle"].items() if k in SIGNALS})
            for key in ("manual_inhibit", "actuator_power", "can_silent", "controller_dead"):
                if key in msg:
                    setattr(s, key, bool(msg[key]))
            if "stale" in msg:
                s.stale = {x for x in msg["stale"] if x in SIGNALS}
            if "speed_nan" in msg:
                if msg["speed_nan"]:
                    s.override["speed"] = math.nan
                else:
                    s.override.pop("speed", None)
            for ch, faults in msg.get("channels", {}).items():
                if ch in CHANNELS:
                    for f, v in faults.items():
                        if f in PLANT_FAULTS:
                            setattr(s.plant(ch), f, bool(v))
            if msg.get("clear_injections"):
                s.manual_inhibit, s.actuator_power, s.can_silent, s.controller_dead = False, True, False, False
                s.stale.clear()
                s.override.clear()
                for p in s.plants:
                    p.jammed = p.sensor_failed = p.comms_lost = p.supply_lost = False
            if msg.get("fault_reset"):
                s.fault_reset = True
            if msg.get("service_exit"):
                s.service_exit = True

    def playback(self, msg: dict) -> None:
        with self.lock:
            if "paused" in msg:
                self.paused = bool(msg["paused"])
                self._last_wall = time.monotonic()
            if "speed" in msg and float(msg["speed"]) in SPEEDS:
                self.speed = float(msg["speed"])

    # ------------------------------------------------------------ state
    def snapshot(self) -> dict:
        with self.lock:
            s = self.sim
            row = s.row() if s.out else {}
            lap_t = max(0.0, (s.now - GRID_MS) / 1000.0) if self.mode == "lap" else 0.0
            st = self.stats
            return _clean({
                "row": row,
                "mode": self.mode,
                "paused": self.paused,
                "speed": self.speed,
                "history": list(self.history),
                "transitions": list(self.transitions)[::-1],
                "channels": list(CHANNELS[: s.n_ch]),
                "plant_faults": {c: {f: getattr(s.plant(c), f) for f in PLANT_FAULTS} for c in CHANNELS[: s.n_ch]},
                "controls": {
                    "vehicle": s.vehicle, "manual_inhibit": s.manual_inhibit,
                    "actuator_power": s.actuator_power, "can_silent": s.can_silent,
                    "controller_dead": s.controller_dead, "stale": sorted(s.stale),
                    "speed_nan": "speed" in s.override,
                },
                "lap": {
                    "phase": self.lap_phase,
                    "t_s": min(lap_t, self.lap.duration_s),
                    "duration_s": self.lap.duration_s,
                    "idx": self.lap_idx // TRACK_STRIDE,
                    "label": self.lap.label[self.lap_idx],
                    "heading": self._heading(),
                    "ld_pct": 100.0 * st["ld_ms"] / st["lap_ms"] if st["lap_ms"] else 0.0,
                    "ld_entries": st["ld_entries"], "fallbacks": st["fallbacks"], "inhibits": st["inhibits"],
                },
                "trail": self.trail if self.mode == "lap" else [],
                "config": {k: v for k, v in self.config.items() if not k.startswith("_") and k != "plant"},
            })

    def _heading(self) -> float:
        i = self.lap_idx
        j = min(i + 4, len(self.lap.x) - 1)
        i = max(0, j - 4)
        return math.degrees(math.atan2(self.lap.y[j] - self.lap.y[i], self.lap.x[j] - self.lap.x[i]))


def make_handler(live: LiveSim):
    def page() -> bytes:  # re-read each time so HTML edits show on refresh
        return (HERE / "index.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # keep the terminal quiet
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj) -> None:
            self._send(200, json.dumps(obj).encode(), "application/json")

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(200, page(), "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._json(live.snapshot())
            elif self.path == "/api/track":
                self._json(live.track)
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            try:
                msg = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                self._send(400, b"bad json", "text/plain")
                return
            try:
                if self.path == "/api/control":
                    live.control(msg)
                elif self.path == "/api/mode":
                    live.restart(mode=msg.get("mode", "manual"), service_mode=bool(msg.get("service_mode")))
                elif self.path == "/api/playback":
                    live.playback(msg)
                else:
                    self._send(404, b"not found", "text/plain")
                    return
            except (TypeError, ValueError, KeyError, AttributeError) as e:
                self._send(400, f"bad request: {e}".encode(), "text/plain")
                return
            self._json({})

    return Handler


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8050)
    ap.add_argument("--config", default="simulation_example_only.json")
    args = ap.parse_args()

    live = LiveSim(args.config)

    def loop():
        # Keep the simulation thread alive no matter what: a crash here would
        # silently freeze the dashboard. Log once per distinct error instead.
        last_err = None
        while True:
            try:
                live.advance()
                last_err = None
            except Exception as e:  # noqa: BLE001
                if repr(e) != last_err:
                    last_err = repr(e)
                    print(f"simulation error (dashboard keeps running; try Restart): {e!r}", file=sys.stderr, flush=True)
                time.sleep(0.5)
            time.sleep(0.01)

    threading.Thread(target=loop, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(live))
    print(f"Active aero dashboard (SIMULATION ONLY): http://127.0.0.1:{args.port}  (Ctrl+C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
