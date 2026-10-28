"""Synthetic autocross lap: track geometry -> speed profile -> vehicle signals over time.

ARBITRARY NUMBERS. The track layout, grip, power and brake limits are invented
to give a ~50 s lap with realistic-looking straights, slaloms and hairpins.
Replace with logged MoTeC data later (Phase 5 CSV playback).
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

# --- invented vehicle limits (not FSAE car data) ---
LAT_ACCEL_MAX = 14.0     # m/s^2 cornering grip
BRAKE_DECEL_MAX = 13.0   # m/s^2
ACCEL_MAX = 7.5          # m/s^2 at standstill, fades with speed
TOP_SPEED = 27.0         # m/s
WHEELBASE = 1.55         # m
BRAKE_BAR_AT_MAX = 55.0  # brake pressure at max decel
DS = 0.5                 # m, path resolution

# (kind, length_m | (radius_m, angle_deg, +left/-right), label)
LAYOUT = [
    ("straight", 70, "Launch straight"),
    ("arc", (14, 90, -1), "Turn 1"),
    ("straight", 45, "Short chute"),
    ("arc", (11, 45, +1), "Slalom"),
    ("arc", (11, 90, -1), "Slalom"),
    ("arc", (11, 90, +1), "Slalom"),
    ("arc", (11, 45, -1), "Slalom"),
    ("straight", 95, "Back straight"),
    ("arc", (8, 180, -1), "Hairpin"),
    ("straight", 60, "Return chute"),
    ("arc", (20, 60, +1), "Sweeper"),
    ("arc", (20, 60, -1), "Sweeper"),
    ("straight", 85, "Long straight"),
    ("arc", (10, 70, -1), "Chicane"),
    ("arc", (10, 70, +1), "Chicane"),
    ("straight", 40, "Chute"),
    ("arc", (12, 100, -1), "Turn 9"),
    ("straight", 55, "Chute"),
    ("arc", (9, 45, +1), "Slalom 2"),
    ("arc", (9, 90, -1), "Slalom 2"),
    ("arc", (9, 45, +1), "Slalom 2"),
    ("straight", 50, "Chute"),
    ("arc", (16, 80, +1), "Final turn"),
    ("straight", 110, "Finish straight"),
]


@dataclass
class Lap:
    s: list[float]          # distance along path, m
    x: list[float]
    y: list[float]
    kappa: list[float]      # signed curvature, 1/m (+ = left)
    v: list[float]          # speed, m/s
    t: list[float]          # time at each point, s
    label: list[str]
    duration_s: float
    length_m: float

    def index_at(self, t_s: float) -> int:
        return min(max(bisect.bisect_right(self.t, t_s) - 1, 0), len(self.t) - 1)

    def signals_at(self, t_s: float) -> dict[str, float]:
        """Vehicle signals at lap time t (same names/units as the sim)."""
        i = self.index_at(t_s)
        j = min(i + 1, len(self.t) - 1)
        v, k = self.v[i], self.kappa[i]
        dt = max(self.t[j] - self.t[i], 1e-6)
        a = (self.v[j] - v) / dt
        if a < -0.5:
            brake = min(BRAKE_BAR_AT_MAX, -a / BRAKE_DECEL_MAX * BRAKE_BAR_AT_MAX)
            throttle = 0.0
        elif a > 0.5:
            brake, throttle = 0.0, 1.0
        else:
            brake, throttle = 0.0, 0.25 + 0.35 * min(1.0, v / TOP_SPEED) + (0.4 if abs(k) < 1e-3 else 0.0)
        return {
            "speed": v,
            "throttle": min(throttle, 1.0),
            "brake": brake,
            "steering": math.atan(WHEELBASE * k),
            "yaw_rate": v * k,
            "lat_accel": v * v * k,
        }


def _smooth(vals: list[float], window: int) -> list[float]:
    out, n = [], len(vals)
    for i in range(n):
        lo, hi = max(0, i - window), min(n, i + window + 1)
        out.append(sum(vals[lo:hi]) / (hi - lo))
    return out


def build_lap() -> Lap:
    x, y, h = 0.0, 0.0, 0.0
    xs, ys, ks, labels = [x], [y], [0.0], [LAYOUT[0][2]]
    for kind, spec, label in LAYOUT:
        if kind == "straight":
            n, k = int(spec / DS), 0.0
        else:
            r, ang, d = spec
            n, k = int(r * math.radians(ang) / DS), d / r
        for _ in range(n):
            h += k * DS
            x += DS * math.cos(h)
            y += DS * math.sin(h)
            xs.append(x); ys.append(y); ks.append(k); labels.append(label)

    ks = _smooth(ks, 6)  # ~3 m of steering transition
    n = len(xs)
    s = [i * DS for i in range(n)]

    vlim = [min(TOP_SPEED, math.sqrt(LAT_ACCEL_MAX / abs(k))) if abs(k) > 1e-4 else TOP_SPEED for k in ks]
    v = [0.0] * n
    for i in range(1, n):  # forward pass: traction / power limited
        a = ACCEL_MAX * max(0.15, 1 - v[i - 1] / (TOP_SPEED * 1.15))
        v[i] = min(vlim[i], math.sqrt(v[i - 1] ** 2 + 2 * a * DS))
    for i in range(n - 2, -1, -1):  # backward pass: braking
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * BRAKE_DECEL_MAX * DS))
    v[0] = 0.0

    t = [0.0]
    for i in range(1, n):
        t.append(t[-1] + DS / max((v[i] + v[i - 1]) / 2, 0.5))
    return Lap(s, xs, ys, ks, v, t, labels, t[-1], s[-1])


if __name__ == "__main__":
    lap = build_lap()
    print(f"length {lap.length_m:.0f} m, lap time {lap.duration_s:.1f} s, "
          f"top {max(lap.v) * 3.6:.0f} km/h, min corner {min(v for v in lap.v[50:]) * 3.6:.0f} km/h")
