"""Manual inhibit, stale / invalid signals, and partial vs complete CAN outage."""
import pytest
from conftest import STRAIGHT, to_hdf, to_ld


def _mid_transition(sim):
    """HDF -> LD has been commanded but the elements have not arrived yet."""
    to_hdf(sim)
    sim.set_vehicle(**STRAIGHT)
    sim.run_until(lambda s: s.state == "REDUCED_DRAG", 2000)
    sim.run(30)
    assert sim.channel_status("REAR_MAIN") == "IN_MOTION"
    return sim


@pytest.mark.parametrize("setup", [to_hdf, to_ld, _mid_transition], ids=["in_HDF", "in_LD", "mid_transition"])
def test_manual_inhibit_requests_hdf_and_tracks_confirmation(make_sim, setup):
    sim = setup(make_sim())
    sim.manual_inhibit = True
    sim.step()
    assert sim.state == "INHIBITED" and sim.reason == "MANUAL_INHIBIT"
    assert sim.out.target == 1  # HDF
    # The command alone is not proof: confirmation only once the sensors agree.
    assert sim.run_until(lambda s: s.out.output_confirmed, 1000)
    sim.run(2000)
    assert sim.state == "INHIBITED", "straight-line conditions must not override the switch"


def test_manual_inhibit_release_goes_through_wait(make_sim, cfg):
    sim = to_ld(make_sim())
    sim.manual_inhibit = True
    sim.run(500)
    sim.manual_inhibit = False
    sim.step()
    assert sim.state == "WAIT_FOR_VALID_INPUTS" and sim.reason == "INHIBIT_CLEARED"
    sim.run(cfg["inputs_stable_ms"] + 10)
    assert sim.state == "HIGH_DOWNFORCE"


def test_manual_inhibit_from_wait(make_sim):
    sim = make_sim()
    sim.manual_inhibit = True
    sim.set_vehicle(**STRAIGHT)
    sim.run(1000)
    assert sim.state == "INHIBITED"
    assert all(r["state"] != "REDUCED_DRAG" for r in sim.trace)


def test_single_stale_signal_inhibits_with_its_own_reason(make_sim, cfg):
    sim = to_ld(make_sim())
    sim.stale.add("steering")  # stops updating, last value still plausible (0.0)
    timeout = cfg["signals"]["steering"]["timeout_ms"]
    sim.run(timeout)
    assert sim.state == "REDUCED_DRAG", "not stale until the timeout has passed"
    sim.run(10)
    assert sim.state == "INHIBITED" and sim.reason == "REQUIRED_INPUT_INVALID"
    assert sim.trace[-1]["inhibit"] == "STEERING_INVALID"
    assert sim.trace[-1]["steering_status"] == "STALE"
    sim.stale.clear()
    sim.step()
    assert sim.state == "WAIT_FOR_VALID_INPUTS"
    assert sim.run_until(lambda s: s.state == "HIGH_DOWNFORCE", 1000)


def test_complete_can_outage_is_a_recoverable_fallback(make_sim, cfg):
    sim = to_ld(make_sim())
    sim.can_silent = True
    sim.run_until(lambda s: s.state != "REDUCED_DRAG", 1000)
    # Signal timeouts (50 ms) fire before the bus timeout (100 ms) ...
    assert sim.state == "INHIBITED"
    sim.run(cfg["can_bus_timeout_ms"])
    # ... then the whole-bus fault takes priority.
    assert sim.state == "COMMANDED_FALLBACK" and sim.reason == "CAN_BUS_SILENT"
    assert sim.trace[-1]["latched_faults"] == "", "CAN silence must not latch"
    sim.can_silent = False
    sim.step()
    assert sim.reason == "FAULT_RECOVERING"
    sim.run(cfg["fault_recovery_stable_ms"])
    assert sim.state == "WAIT_FOR_VALID_INPUTS"
    assert sim.run_until(lambda s: s.state == "HIGH_DOWNFORCE", 1000)


def test_can_flicker_restarts_recovery_window(make_sim, cfg):
    sim = to_hdf(make_sim())
    sim.can_silent = True
    sim.run(300)
    sim.can_silent = False
    sim.run(cfg["fault_recovery_stable_ms"] // 2)
    sim.can_silent = True
    sim.run(300)
    sim.can_silent = False
    sim.run(cfg["fault_recovery_stable_ms"] // 2 + 50)
    assert sim.state == "COMMANDED_FALLBACK", "recovery window must restart after a new dropout"


@pytest.mark.parametrize("inject,status", [
    (lambda s: s.override.__setitem__("speed", float("nan")), "FRESH_IMPLAUSIBLE"),
    (lambda s: s.override.__setitem__("speed", 500.0), "FRESH_IMPLAUSIBLE"),
    (lambda s: s.source_invalid.add("speed"), "SOURCE_INVALID"),
], ids=["nan", "out_of_range", "source_invalid"])
def test_invalid_speed_is_not_treated_as_a_number(make_sim, inject, status):
    sim = to_ld(make_sim())
    inject(sim)
    sim.step()
    assert sim.trace[-1]["speed_status"] == status
    assert sim.state == "INHIBITED"
    assert "SPEED_INVALID" in sim.trace[-1]["inhibit"]
