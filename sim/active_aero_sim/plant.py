"""Demonstration-only actuator plant (one per aero channel).

SYNTHETIC MODEL. A rate-limited position with a command delay. It is not
bench-characterized and says nothing about real servo/BLDC behavior.

Deliberately conservative physics:
  - with no supply, no comms, a jam, or no command, the element does NOT move.
    Unpowered backdrive / spring return is unknown, so it is never invented.
  - the output sensor reports the true element position unless failed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class ChannelPlant:
    name: str
    position_deg: float = 0.0
    max_rate_dps: float = 150.0
    command_delay_ms: int = 10

    # injectable faults
    jammed: bool = False
    sensor_failed: bool = False
    comms_lost: bool = False
    supply_lost: bool = False  # this branch only

    _last_cmd: float = field(default=math.nan, repr=False)
    _cmd_since_ms: int = field(default=0, repr=False)

    def sensor_reading(self) -> tuple[float, bool]:
        """(measured_output_deg, output_sensor_ok) as the controller would see it."""
        if self.sensor_failed:
            return math.nan, False
        return self.position_deg, True

    def step(self, command_deg: float, command_enabled: bool, system_supply_ok: bool,
             now_ms: int, dt_ms: int) -> None:
        if math.isnan(command_deg) or not command_enabled:
            self._last_cmd = math.nan
            return
        if command_deg != self._last_cmd:
            self._last_cmd = command_deg
            self._cmd_since_ms = now_ms
        powered = system_supply_ok and not self.supply_lost
        if not powered or self.comms_lost or self.jammed:
            return
        if now_ms - self._cmd_since_ms < self.command_delay_ms:
            return
        max_step = self.max_rate_dps * dt_ms / 1000.0
        err = command_deg - self.position_deg
        self.position_deg += max(-max_step, min(max_step, err))
