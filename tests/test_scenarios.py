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
