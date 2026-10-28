import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))

from active_aero_sim.invariants import check as check_invariants  # noqa: E402
from active_aero_sim.scenario import load_config  # noqa: E402
from active_aero_sim.sim import Sim  # noqa: E402

# Synthetic, simulation-only thresholds (see config/simulation_example_only.json).
SIM_CONFIG = "simulation_example_only.json"

STRAIGHT = dict(speed=30.0, throttle=1.0, brake=0.0, steering=0.0, yaw_rate=0.0, lat_accel=0.0)
CRUISE = dict(speed=10.0, throttle=0.3, brake=0.0, steering=0.0, yaw_rate=0.0, lat_accel=0.0)


@pytest.fixture
def cfg():
    return load_config(SIM_CONFIG)


@pytest.fixture
def make_sim(cfg):
    sims = []

    def _make(**kw):
        sim = Sim(cfg, **kw)
        sims.append(sim)
        return sim

    yield _make
    # Every simulated trace in the suite must satisfy the core invariants.
    for sim in sims:
        violations = check_invariants(sim.trace, cfg)
        assert not violations, "invariant violations:\n" + "\n".join(violations[:10])


def to_hdf(sim, **vehicle):
    sim.set_vehicle(**(vehicle or CRUISE))
    assert sim.run_until(lambda s: s.state == "HIGH_DOWNFORCE", 2000), sim.reason
    return sim


def to_ld(sim):
    to_hdf(sim)
    sim.set_vehicle(**STRAIGHT)
    assert sim.run_until(lambda s: s.state == "REDUCED_DRAG", 2000), sim.reason
    assert sim.run_until(lambda s: s.out.output_confirmed, 2000)
    return sim
