"""Every scripted scenario meets its expectations and the core invariants, deterministically."""
import pytest

from active_aero_sim.scenario import SCENARIO_DIR, load_scenario, run

SCENARIOS = sorted(SCENARIO_DIR.glob("*.json"))


@pytest.mark.parametrize("path", SCENARIOS, ids=[p.stem for p in SCENARIOS])
def test_scenario(path):
    _, result = run(load_scenario(path))
    problems = result["expectation_failures"] + result["invariant_violations"]
    assert result["passed"], "\n".join(problems)


@pytest.mark.parametrize("path", SCENARIOS, ids=[p.stem for p in SCENARIOS])
def test_replay_is_deterministic(path):
    sc = load_scenario(path)
    a, _ = run(sc)
    b, _ = run(sc)
    assert repr(a.trace) == repr(b.trace)  # repr so NaN == NaN


def test_autocross_lap_uses_ld_on_straights_only():
    sim, result = run(load_scenario(SCENARIO_DIR / "09_autocross_lap.json"))
    ld_entries = [t for t in result["transitions"] if t["state"] == "REDUCED_DRAG"]
    assert len(ld_entries) >= 5, "LD should open on most straights"
    exits = [t["reason"] for t in result["transitions"][1:] if t["state"] == "HIGH_DOWNFORCE" and t["reason"] != "INPUTS_VALID"]
    assert exits and all(r.startswith("STABILITY_") or r == "LD_CONDITIONS_LOST" for r in exits)
