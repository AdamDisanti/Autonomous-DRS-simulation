"""Boot, wait, HDF/LD eligibility, priority and hysteresis (normal operation paths)."""
import pytest
from conftest import CRUISE, STRAIGHT, to_hdf, to_ld

from active_aero_sim.scenario import load_config
from active_aero_sim.sim import Sim


def test_tbd_config_refuses_to_leave_init():
    sim = Sim(load_config("controls_thresholds.json"))
    sim.set_vehicle(**STRAIGHT)
    sim.run(1000)
    assert sim.state == "POWER_ON_INIT"
    assert sim.reason == "CONFIG_INCOMPLETE"
    assert not sim.out.command_enabled


def test_variable_aero_enabled_config_is_rejected(cfg):
    cfg = dict(cfg, variable_aero_enabled=True)
    sim = Sim(cfg)
    sim.run(500)
    assert sim.state == "POWER_ON_INIT"


def test_boot_with_no_can_waits_and_commands_nothing(make_sim):
    sim = make_sim()
    sim.can_silent = True  # nothing ever received
    sim.run(2000)
    assert sim.state == "WAIT_FOR_VALID_INPUTS"
    assert sim.reason == "WAITING_FOR_INPUTS"
    assert sim.out.target == 0 and not sim.out.command_enabled
    assert {r["state"] for r in sim.trace} == {"WAIT_FOR_VALID_INPUTS"}


def test_valid_inputs_enter_hdf_first_even_on_a_straight(make_sim, cfg):
    sim = make_sim()
    sim.set_vehicle(**STRAIGHT)  # LD-eligible from the very first tick
    sim.run_until(lambda s: s.state != "WAIT_FOR_VALID_INPUTS", 2000)
    assert sim.state == "HIGH_DOWNFORCE"
    assert sim.now >= cfg["inputs_stable_ms"]
    entered = sim.now
    sim.run_until(lambda s: s.state == "REDUCED_DRAG", 2000)
    assert sim.now - entered >= cfg["ld_entry_debounce_ms"]


def test_ld_requires_continuous_debounce(make_sim, cfg):
    sim = to_hdf(make_sim())
    sim.set_vehicle(**STRAIGHT)
    sim.run(cfg["ld_entry_debounce_ms"] - 20)
    assert sim.state == "HIGH_DOWNFORCE" and sim.reason == "LD_DEBOUNCING"
    sim.set_vehicle(throttle=0.5)  # interruption restarts the debounce
    sim.step()
    sim.set_vehicle(throttle=1.0)
    sim.run(cfg["ld_entry_debounce_ms"] - 20)
    assert sim.state == "HIGH_DOWNFORCE"
    sim.run(40)
    assert sim.state == "REDUCED_DRAG"


def test_braking_exits_ld_on_the_same_tick(make_sim):
    sim = to_ld(make_sim())
    sim.set_vehicle(brake=30.0)
    sim.step()
    assert sim.state == "HIGH_DOWNFORCE"
    assert sim.reason == "STABILITY_BRAKING"


@pytest.mark.parametrize("signal,value,reason", [
    ("steering", 0.3, "STABILITY_STEERING"),
    ("steering", -0.3, "STABILITY_STEERING"),
    ("yaw_rate", 0.6, "STABILITY_YAW"),
    ("lat_accel", -8.0, "STABILITY_LAT_ACCEL"),
])
def test_cornering_beats_full_throttle(make_sim, signal, value, reason):
    sim = to_ld(make_sim())
    sim.set_vehicle(**{signal: value})  # throttle still 1.0
    sim.step()
    assert sim.state == "HIGH_DOWNFORCE" and sim.reason == reason
    sim.run(1000)
    assert sim.state == "HIGH_DOWNFORCE"


def test_hysteresis_band(make_sim, cfg):
    mid = (cfg["ld_entry_speed_mps"] + cfg["ld_exit_speed_mps"]) / 2
    sim = to_hdf(make_sim())
    sim.set_vehicle(speed=mid, throttle=1.0)
    sim.run(1000)
    assert sim.state == "HIGH_DOWNFORCE", "must not enter LD inside the hysteresis band"
    sim = to_ld(make_sim())
    sim.set_vehicle(speed=mid)
    sim.run(1000)
    assert sim.state == "REDUCED_DRAG", "must not leave LD inside the hysteresis band"
    sim.set_vehicle(speed=cfg["ld_exit_speed_mps"] - 0.5)
    sim.step()
    assert sim.state == "HIGH_DOWNFORCE" and sim.reason == "LD_CONDITIONS_LOST"


def test_noisy_speed_near_threshold_does_not_chatter(make_sim, cfg):
    sim = to_hdf(make_sim(seed=7))
    sim.set_vehicle(speed=cfg["ld_entry_speed_mps"], throttle=1.0)
    sim.noise_std["speed"] = 0.8
    start = sim.out.transition_count
    sim.run(5000)
    assert sim.out.transition_count - start <= 2


def test_variable_aero_is_never_entered(make_sim):
    sim = to_ld(make_sim())
    sim.run(2000)
    assert all(r["state"] != "INTERMEDIATE_VARIABLE_AERO" for r in sim.trace)


def test_service_mode_entry_and_exit(make_sim):
    sim = make_sim(service_mode=True)
    sim.set_vehicle(**STRAIGHT)
    sim.run(1000)
    assert sim.state == "SERVICE_CALIBRATION"
    assert not sim.out.command_enabled
    sim.service_exit = True
    sim.step()
    assert sim.state == "WAIT_FOR_VALID_INPUTS" and sim.reason == "SERVICE_EXIT"
    sim.run_until(lambda s: s.state != "WAIT_FOR_VALID_INPUTS", 2000)
    assert sim.state == "HIGH_DOWNFORCE", "service exit must not jump straight to LD"


def test_service_request_after_boot_is_ignored(make_sim):
    sim = to_hdf(make_sim())
    sim.service_request = True
    sim.run(200)
    assert sim.state != "SERVICE_CALIBRATION"


def test_uint32_clock_wraparound(make_sim, cfg):
    sim = make_sim(start_ms=2**32 - 300)
    sim.set_vehicle(**CRUISE)
    sim.run(1000)  # crosses the uint32 wrap
    assert sim.state == "HIGH_DOWNFORCE"
    sim.set_vehicle(**STRAIGHT)
    sim.run(cfg["ld_entry_debounce_ms"] + 20)
    assert sim.state == "REDUCED_DRAG"
