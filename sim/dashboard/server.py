"""Live interactive dashboard for the active aero supervisor simulation.

    python sim/dashboard/server.py            # then open http://127.0.0.1:8050

A background thread runs the Sim (C core + synthetic plant) in real time.
The browser polls /api/state and posts control changes to /api/control.
Standard library only. SIMULATION ONLY.
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

HISTORY_MS = 10_000
HISTORY_EVERY_MS = 20
PLANT_FAULTS = ("jammed", "sensor_failed", "comms_lost", "supply_lost")


def _clean(v):
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return None
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    return v


class LiveSim:
    def __init__(self, config_name: str):
        self.config_name = config_name
        self.lock = threading.Lock()
        self.restart(service_mode=False)

    def restart(self, service_mode: bool) -> None:
        with self.lock:
            self.config = load_config(self.config_name)
            self.sim = Sim(self.config, tick_ms=5, service_mode=service_mode)
            self.sim.set_vehicle(speed=0, throttle=0, brake=0, steering=0, yaw_rate=0, lat_accel=0)
            self.history: deque[dict] = deque(maxlen=HISTORY_MS // HISTORY_EVERY_MS)
            self.transitions: deque[dict] = deque(maxlen=40)
            self._last_state = None
            self._wall0 = time.monotonic()

    def advance(self) -> None:
        with self.lock:
            target_ms = int((time.monotonic() - self._wall0) * 1000)
            steps = 0
            while self.sim.now + self.sim.tick_ms <= target_ms and steps < 400:
                self.sim.step()
                steps += 1
                r = self.sim.trace[-1]
                if r["state"] != self._last_state:
                    self.transitions.append({"t_ms": r["t_ms"], "state": r["state"], "reason": r["reason"]})
                    self._last_state = r["state"]
                if r["t_ms"] % HISTORY_EVERY_MS == 0:
                    self.history.append({
                        "t": r["t_ms"], "state": r["state"],
                        "pos": [r[f"{c}_true_pos"] for c in CHANNELS[: self.sim.n_ch]],
                        "cmd": [r[f"{c}_cmd"] for c in CHANNELS[: self.sim.n_ch]],
                    })
            if steps >= 400:  # fell behind (e.g. laptop asleep): resync instead of catching up
                self._wall0 = time.monotonic() - self.sim.now / 1000
            self.sim.trace.clear()  # live mode keeps only the rolling history

    def control(self, msg: dict) -> None:
        with self.lock:
            s = self.sim
            if "vehicle" in msg:
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
            if msg.get("fault_reset"):
                s.fault_reset = True
            if msg.get("service_exit"):
                s.service_exit = True

    def snapshot(self) -> dict:
        with self.lock:
            s = self.sim
            row = s.row() if s.out else {}
            return _clean({
                "row": row,
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
                "config": {k: v for k, v in self.config.items() if not k.startswith("_") and k != "plant"},
                "config_name": self.config_name,
            })


def make_handler(live: LiveSim):
    index = (HERE / "index.html").read_bytes()

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

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(200, index, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, json.dumps(live.snapshot()).encode(), "application/json")
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            try:
                msg = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                self._send(400, b"bad json", "text/plain")
                return
            if self.path == "/api/control":
                live.control(msg)
            elif self.path == "/api/restart":
                live.restart(service_mode=bool(msg.get("service_mode")))
            else:
                self._send(404, b"not found", "text/plain")
                return
            self._send(200, b"{}", "application/json")

    return Handler


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8050)
    ap.add_argument("--config", default="simulation_example_only.json")
    args = ap.parse_args()

    live = LiveSim(args.config)

    def loop():
        while True:
            live.advance()
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
