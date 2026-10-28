"""Run scripted scenarios through the C supervisor and write traces, plots and a summary.

    python scripts/run_scenario.py                 # all scenarios in sim/scenarios/
    python scripts/run_scenario.py 04_power_cut    # any scenario whose file name contains this
    python scripts/run_scenario.py --no-plot

Outputs go to results/<scenario_id>/ (trace.csv, result.json, timeline.png).
SIMULATION ONLY: synthetic thresholds and actuator physics.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))

from active_aero_sim.scenario import SCENARIO_DIR, load_scenario, run  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("filter", nargs="?", default="", help="substring of scenario file name")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--out", default=str(ROOT / "results"))
    args = ap.parse_args()

    files = sorted(p for p in SCENARIO_DIR.glob("*.json") if args.filter in p.stem)
    if not files:
        print(f"no scenarios match {args.filter!r}")
        return 2

    all_passed = True
    for path in files:
        sc = load_scenario(path)
        sim, result = run(sc)
        out = Path(args.out) / sc["scenario_id"]
        out.mkdir(parents=True, exist_ok=True)
        with open(out / "trace.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(sim.trace[0]))
            w.writeheader()
            w.writerows(sim.trace)
        (out / "result.json").write_text(json.dumps(result, indent=2))
        if not args.no_plot:
            from active_aero_sim.plot import plot_trace
            plot_trace(sim.trace, sc["scenario_id"], out / "timeline.png", sim.n_ch)

        mark = "PASS" if result["passed"] else "FAIL"
        all_passed &= result["passed"]
        print(f"[{mark}] {sc['scenario_id']}")
        for t in result["transitions"]:
            print(f"        {t['t_ms']:>6} ms  {t['state']:<28} ({t['reason']})")
        for msg in result["expectation_failures"] + result["invariant_violations"]:
            print(f"   !!  {msg}")
    print(f"\nResults in {args.out}/")
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
