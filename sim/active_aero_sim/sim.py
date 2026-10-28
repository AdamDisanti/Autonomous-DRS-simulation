"""Deterministic software-in-the-loop simulation around the C supervisor.

One Sim = one simulated car: synthetic vehicle signals in, the C core makes
every decision, synthetic actuator plants respond. Time is a simulated
monotonic millisecond clock; randomness (sensor noise) comes from a seeded RNG,
so the same inputs always give the same trace.

CAN here is a placeholder: signals are "received" every tick unless a silence
or stale fault is injected. Real message IDs / framing are TBD (Phase 2).
"""
from __future__ import annotations

import math
import random

from .core import (CHANNELS, FAULTS, MAX_CHANNELS, POS_STATUS, REASONS, SIGNAL_STATUS, SIGNALS,
                   STATES, TARGETS, Inputs, Output, Supervisor, config_from_dict, fault_names,
                   inhibit_names)
from .plant import ChannelPlant

U32 = 0xFFFFFFFF


class Sim:
    def __init__(self, config: dict, tick_ms: int = 5, seed: int = 0, service_mode: bool = False,
                 start_ms: int = 0):
        self.config = config
        self.tick_ms = tick_ms
        self.cfg = config_from_dict(config)
        self.sup = Supervisor(self.cfg, start_ms & U32)
        self.n_ch = int(config.get("channels", {}).get("count") or 3)
        self.rng = random.Random(seed)

        plant_cfg = config.get("plant", {})
        rates = plant_cfg.get("max_rate_dps", [150.0] * MAX_CHANNELS)
        delays = plant_cfg.get("command_delay_ms", [10] * MAX_CHANNELS)
        init = plant_cfg.get("initial_position_deg", [0.0] * MAX_CHANNELS)
        self.plants = [
            ChannelPlant(CHANNELS[i], position_deg=init[i], max_rate_dps=rates[i], command_delay_ms=delays[i])
            for i in range(MAX_CHANNELS)
        ]

        # vehicle signal "truth" and injectable signal faults
        self.vehicle = {name: 0.0 for name in SIGNALS}
        self.noise_std: dict[str, float] = {}
        self.can_silent = False
        self.stale: set[str] = set()           # signal stops updating, keeps last value
        self.source_invalid: set[str] = set()  # sender flags the value invalid
        self.override: dict[str, float] = {}   # e.g. NaN or out-of-range values

        # switches / power
        self.manual_inhibit = False
        self.actuator_power = True
        self.controller_dead = False           # sim-only: MCU stopped, no software runs
        self.service_request = service_mode    # only honored on the first tick (POWER_ON_INIT)
        self.service_exit = False              # one-tick pulse
        self.fault_reset = False               # one-tick pulse

        self.now = start_ms  # unbounded sim time; the C core sees it wrapped to uint32
        self.inputs = Inputs()
        self.out: Output | None = None
        self.trace: list[dict] = []

    # ------------------------------------------------------------ helpers
    def plant(self, channel: str) -> ChannelPlant:
        return self.plants[CHANNELS.index(channel)]

    @property
    def state(self) -> str:
        return STATES[self.out.state] if self.out else "POWER_ON_INIT"

    @property
    def reason(self) -> str:
        return REASONS[self.out.reason] if self.out else "NONE"

    def channel_status(self, channel: str) -> str:
        return POS_STATUS[self.out.channel_status[CHANNELS.index(channel)]]

    def set_vehicle(self, **values: float) -> None:
        for k, v in values.items():
            if k not in self.vehicle:
                raise KeyError(f"unknown signal {k!r}; expected one of {SIGNALS}")
            self.vehicle[k] = float(v)

    # ------------------------------------------------------------ stepping
    def step(self) -> Output:
        self.now += self.tick_ms
        now32 = self.now & U32
        inp = self.inputs

        if not self.can_silent:
            for i, name in enumerate(SIGNALS):
                if name in self.stale:
                    continue
                v = self.override.get(name, self.vehicle[name])
                std = self.noise_std.get(name)
                if std and not math.isnan(v):
                    v += self.rng.gauss(0.0, std)
                s = inp.signal[i]
                s.value = v
                s.timestamp_ms = now32
                s.received = True
                s.source_valid = name not in self.source_invalid
            inp.can_last_rx_ms = now32
            inp.can_ever_rx = True

        inp.manual_inhibit = self.manual_inhibit
        inp.actuator_supply_good = self.actuator_power
        inp.service_request = self.service_request
        inp.service_exit = self.service_exit
        inp.fault_reset_request = self.fault_reset
        for i, p in enumerate(self.plants):
            meas, ok = p.sensor_reading()
            inp.channel[i].measured_output_deg = meas
            inp.channel[i].output_sensor_ok = ok
            inp.channel[i].comms_ok = not (p.comms_lost or p.supply_lost)

        if not self.controller_dead or self.out is None:
            self.out = self.sup.tick(inp, now32)
        command_enabled = self.out.command_enabled and not self.controller_dead
        for i in range(self.n_ch):
            self.plants[i].step(self.out.channel_command_deg[i], command_enabled,
                                self.actuator_power, self.now, self.tick_ms)

        self.trace.append(self.row())
        self.service_request = False
        self.service_exit = False
        self.fault_reset = False
        return self.out

    def run(self, duration_ms: int) -> Output:
        for _ in range(max(1, duration_ms // self.tick_ms)):
            self.step()
        return self.out

    def run_until(self, predicate, timeout_ms: int) -> bool:
        """Step until predicate(sim) is true; False if it never happens."""
        for _ in range(timeout_ms // self.tick_ms):
            self.step()
            if predicate(self):
                return True
        return False

    # ------------------------------------------------------------ logging
    def row(self) -> dict:
        o = self.out
        r = {
            "t_ms": self.now,
            "state": STATES[o.state],
            "reason": REASONS[o.reason],
            "target": TARGETS[o.target],
            "command_enabled": bool(o.command_enabled) and not self.controller_dead,
            "output_confirmed": bool(o.output_confirmed),
            "inhibit": "|".join(inhibit_names(o.inhibit_flags)),
            "active_faults": "|".join(fault_names(o.active_faults)),
            "latched_faults": "|".join(fault_names(o.latched_faults)),
            "transition_count": o.transition_count,
            "manual_inhibit": self.manual_inhibit,
            "actuator_power": self.actuator_power,
            "can_silent": self.can_silent,
            "controller_dead": self.controller_dead,
        }
        for i, name in enumerate(SIGNALS):
            r[name] = self.inputs.signal[i].value if self.inputs.signal[i].received else math.nan
            r[f"{name}_status"] = SIGNAL_STATUS[o.signal_status[i]]
        for i in range(self.n_ch):
            ch = CHANNELS[i]
            fb = self.inputs.channel[i]  # what the controller saw this tick
            r[f"{ch}_cmd"] = o.channel_command_deg[i]
            r[f"{ch}_true_pos"] = self.plants[i].position_deg  # sim-only ground truth
            r[f"{ch}_sensor"] = fb.measured_output_deg if fb.output_sensor_ok else math.nan
            r[f"{ch}_status"] = POS_STATUS[o.channel_status[i]]
        return r


__all__ = ["Sim", "FAULTS", "CHANNELS", "SIGNALS", "STATES"]
