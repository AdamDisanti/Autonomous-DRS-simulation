"""Timeline plot of a simulated trace (state, inputs, commanded vs measured position, faults)."""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .core import CHANNELS, STATES  # noqa: E402

STATE_ORDER = list(STATES)
CH_COLORS = {"FRONT_LEFT": "#2a78c4", "FRONT_RIGHT": "#3fa66b", "REAR_MAIN": "#d9822b", "REAR_AUX": "#8a5cc2"}


def plot_trace(trace: list[dict], title: str, path: Path, n_channels: int = 3) -> Path:
    t = [r["t_ms"] / 1000.0 for r in trace]
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True,
                             gridspec_kw={"height_ratios": [1.4, 1.2, 1.6, 0.9]})
    fig.suptitle(f"{title}\nSIMULATION ONLY - synthetic thresholds and actuator model", fontsize=11)

    ax = axes[0]
    ax.step(t, [STATE_ORDER.index(r["state"]) for r in trace], where="post", color="#1d3557", lw=2)
    ax.set_yticks(range(len(STATE_ORDER)), [s.replace("_", " ").title() for s in STATE_ORDER], fontsize=8)
    ax.set_ylabel("state")
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(t, [r["speed"] for r in trace], label="speed [m/s]", color="#1d3557")
    ax.plot(t, [r["brake"] for r in trace], label="brake [bar]", color="#c0392b")
    ax.plot(t, [r["throttle"] * 10 for r in trace], label="throttle x10", color="#27ae60")
    ax.plot(t, [r["steering"] * 100 for r in trace], label="steering [rad] x100", color="#8e44ad", alpha=0.7)
    ax.set_ylabel("inputs")
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[2]
    for ch in CHANNELS[:n_channels]:
        c = CH_COLORS[ch]
        ax.plot(t, [r[f"{ch}_cmd"] for r in trace], color=c, ls="--", lw=1, label=f"{ch} command")
        ax.plot(t, [r[f"{ch}_sensor"] for r in trace], color=c, lw=2, label=f"{ch} measured")
    ax.set_ylabel("element position [deg]")
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[3]
    lanes = [
        ("manual inhibit", lambda r: r["manual_inhibit"]),
        ("actuator power cut", lambda r: not r["actuator_power"]),
        ("CAN silent", lambda r: r["can_silent"]),
        ("inputs invalid", lambda r: bool(r["inhibit"].replace("MANUAL", "").strip("|"))),
        ("fault active", lambda r: bool(r["active_faults"])),
        ("fault latched", lambda r: bool(r["latched_faults"])),
        ("output confirmed", lambda r: r["output_confirmed"]),
    ]
    for i, (label, fn) in enumerate(lanes):
        on = [r["t_ms"] / 1000 for r in trace if fn(r)]
        ax.scatter(on, [i] * len(on), marker="s", s=4, color="#c0392b" if "confirmed" not in label else "#27ae60")
    ax.set_yticks(range(len(lanes)), [l for l, _ in lanes], fontsize=8)
    ax.set_ylim(-0.5, len(lanes) - 0.5)
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


