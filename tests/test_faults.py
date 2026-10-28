"""Power loss, jam / incomplete return, sensor and comms faults, latching and reset."""
from conftest import to_hdf, to_ld


def _positions(sim):
    return [p.position_deg for p in sim.plants[: sim.n_ch]]


def test_power_cut_in_ld_does_not_assume_hdf(make_sim, cfg):
    sim = to_ld(make_sim())
    ld = _positions(sim)
    sim.actuator_power = False
    sim.step()
    assert sim.state == "COMMANDED_FALLBACK" and sim.reason == "ACTUATOR_SUPPLY_LOST"
    assert not sim.out.command_enabled
    sim.run(2000)
    assert _positions(sim) == ld, "plant must not invent a passive return to HDF"
    assert not sim.out.output_confirmed
    assert all(sim.channel_status(c) == "UNCONFIRMED" for c in ("FRONT_LEFT", "FRONT_RIGHT", "REAR_MAIN"))


def test_power_fault_latches_until_deliberate_reset(make_sim, cfg):
    sim = to_ld(make_sim())
    sim.actuator_power = False
    sim.run(200)
    sim.fault_reset = True  # reset while the condition is still present: ignored
    sim.step()
    assert "ACTUATOR_SUPPLY" in sim.trace[-1]["latched_faults"]
    sim.actuator_power = True
    sim.run(2000)
    assert sim.state == "COMMANDED_FALLBACK" and sim.reason == "LATCHED_FAULT"
    assert sim.out.output_confirmed, "with power back, fallback drives and confirms HDF"
    sim.fault_reset = True
    sim.step()
    assert sim.trace[-1]["latched_faults"] == ""
    sim.run(cfg["fault_recovery_stable_ms"])
    assert sim.state == "WAIT_FOR_VALID_INPUTS"


def test_rear_jam_during_return_is_never_confirmed(make_sim, cfg):
    sim = to_ld(make_sim())
    jam_idx = len(sim.trace)
    sim.plant("REAR_MAIN").jammed = True
    sim.set_vehicle(brake=30.0)
    sim.step()
    assert sim.state == "HIGH_DOWNFORCE"
    sim.run(cfg["motion_timeout_ms"] + 20)
    assert sim.channel_status("FRONT_LEFT") == "CONFIRMED"
    assert sim.channel_status("REAR_MAIN") == "UNCONFIRMED"
    assert "MOTION_TIMEOUT" in sim.trace[-1]["latched_faults"]
    assert sim.state == "COMMANDED_FALLBACK"
    assert not sim.out.output_confirmed
    sim.fault_reset = True  # still jammed: reset must not clear it
    sim.run(100)
    assert sim.state == "COMMANDED_FALLBACK" and "MOTION_TIMEOUT" in sim.trace[-1]["latched_faults"]
    assert all(r["REAR_MAIN_status"] != "CONFIRMED" for r in sim.trace[jam_idx:])

    sim.plant("REAR_MAIN").jammed = False
    assert sim.run_until(lambda s: s.channel_status("REAR_MAIN") == "CONFIRMED", 2000)
    sim.step()
    sim.fault_reset = True
    sim.step()
    assert sim.trace[-1]["latched_faults"] == ""
    sim.run(cfg["fault_recovery_stable_ms"] + 10)
    assert sim.state == "WAIT_FOR_VALID_INPUTS"


def test_output_sensor_failure_gives_unknown_not_confirmed(make_sim):
    sim = to_hdf(make_sim())
    sim.plant("FRONT_LEFT").sensor_failed = True
    sim.step()
    assert sim.channel_status("FRONT_LEFT") == "UNKNOWN"
    assert not sim.out.output_confirmed
    sim.step()
    assert sim.state == "COMMANDED_FALLBACK"
    assert "OUTPUT_SENSOR" in sim.trace[-1]["latched_faults"]


def test_one_front_channel_comms_loss_latches(make_sim):
    # NOTE: fallback still commands the healthy channels to HDF, which can leave a
    # left/right asymmetry. Whether that is acceptable is an OPEN team decision.
    sim = to_ld(make_sim())
    sim.plant("FRONT_LEFT").comms_lost = True
    sim.run(500)
    assert sim.state == "COMMANDED_FALLBACK"
    assert "ACTUATOR_COMMS" in sim.trace[-1]["latched_faults"]
    assert sim.channel_status("FRONT_LEFT") != "CONFIRMED"
    assert sim.channel_status("FRONT_RIGHT") == "CONFIRMED"


def test_dead_controller_issues_nothing(make_sim):
    sim = to_ld(make_sim())
    before = _positions(sim)
    frozen_state = sim.state
    sim.controller_dead = True
    sim.set_vehicle(brake=50.0)  # would normally force HDF
    sim.run(2000)
    assert _positions(sim) == before, "no software fallback from a dead MCU"
    assert all(not r["command_enabled"] for r in sim.trace[-400:])
    assert sim.state == frozen_state  # last output, never updated


def test_manual_inhibit_beats_latched_fault(make_sim):
    sim = to_hdf(make_sim())
    sim.plant("REAR_MAIN").comms_lost = True
    sim.run(50)
    assert sim.state == "COMMANDED_FALLBACK"
    sim.manual_inhibit = True
    sim.step()
    assert sim.state == "INHIBITED"
    sim.manual_inhibit = False
    sim.step()
    assert sim.state == "COMMANDED_FALLBACK", "latched fault reappears once inhibit is released"
