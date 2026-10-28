"""ctypes binding to the portable C core (core/src/aero_core.c).

The simulator never re-implements supervisor logic in Python: every decision
comes from the same C code that is intended to move to the ESP32.
Struct layouts here must mirror core/include/aero_core.h; sizes are checked
against the C library at load time.
"""
from __future__ import annotations

import ctypes as C
import math
import os
import sys
from pathlib import Path

MAX_CHANNELS = 4
SIGNALS = ("speed", "throttle", "brake", "steering", "yaw_rate", "lat_accel")
SIGNAL_UNITS = ("m/s", "0..1", "bar", "rad", "rad/s", "m/s^2")
CHANNELS = ("FRONT_LEFT", "FRONT_RIGHT", "REAR_MAIN", "REAR_AUX")
TBD_MS = 0xFFFFFFFF

STATES = (
    "POWER_ON_INIT",
    "SERVICE_CALIBRATION",
    "WAIT_FOR_VALID_INPUTS",
    "HIGH_DOWNFORCE",
    "REDUCED_DRAG",
    "INTERMEDIATE_VARIABLE_AERO",
    "INHIBITED",
    "COMMANDED_FALLBACK",
)
TARGETS = ("NONE", "HDF", "LD")
POS_STATUS = ("NOT_COMMANDED", "REQUESTED", "IN_MOTION", "CONFIRMED", "UNCONFIRMED", "UNKNOWN")
SIGNAL_STATUS = ("NEVER_RECEIVED", "FRESH_VALID", "FRESH_IMPLAUSIBLE", "STALE", "SOURCE_INVALID")

FAULTS = {
    "CAN_BUS_SILENT": 1 << 0,
    "ACTUATOR_SUPPLY": 1 << 1,
    "ACTUATOR_COMMS": 1 << 2,
    "OUTPUT_SENSOR": 1 << 3,
    "MOTION_TIMEOUT": 1 << 4,
}
INHIBIT_MANUAL = 1 << 0


def inhibit_signal_bit(index: int) -> int:
    return 1 << (1 + index)


def fault_names(bits: int) -> list[str]:
    return [name for name, bit in FAULTS.items() if bits & bit]


def inhibit_names(bits: int) -> list[str]:
    names = ["MANUAL"] if bits & INHIBIT_MANUAL else []
    names += [f"{s.upper()}_INVALID" for i, s in enumerate(SIGNALS) if bits & inhibit_signal_bit(i)]
    return names


NSIG = len(SIGNALS)


class SignalSample(C.Structure):
    _fields_ = [
        ("value", C.c_float),
        ("timestamp_ms", C.c_uint32),
        ("received", C.c_bool),
        ("source_valid", C.c_bool),
    ]


class ChannelFeedback(C.Structure):
    _fields_ = [
        ("measured_output_deg", C.c_float),
        ("output_sensor_ok", C.c_bool),
        ("comms_ok", C.c_bool),
    ]


class Config(C.Structure):
    _fields_ = [
        ("signal_required", C.c_bool * NSIG),
        ("signal_timeout_ms", C.c_uint32 * NSIG),
        ("signal_min", C.c_float * NSIG),
        ("signal_max", C.c_float * NSIG),
        ("can_bus_timeout_ms", C.c_uint32),
        ("inputs_stable_ms", C.c_uint32),
        ("fault_recovery_stable_ms", C.c_uint32),
        ("ld_entry_speed_mps", C.c_float),
        ("ld_exit_speed_mps", C.c_float),
        ("ld_entry_throttle", C.c_float),
        ("ld_exit_throttle", C.c_float),
        ("ld_entry_debounce_ms", C.c_uint32),
        ("hdf_brake_bar", C.c_float),
        ("hdf_steering_rad", C.c_float),
        ("hdf_yaw_rate_rps", C.c_float),
        ("hdf_lat_accel_mps2", C.c_float),
        ("channel_count", C.c_uint8),
        ("hdf_position_deg", C.c_float * MAX_CHANNELS),
        ("ld_position_deg", C.c_float * MAX_CHANNELS),
        ("position_tolerance_deg", C.c_float),
        ("motion_timeout_ms", C.c_uint32),
        ("variable_aero_enabled", C.c_bool),
    ]


class Inputs(C.Structure):
    _fields_ = [
        ("signal", SignalSample * NSIG),
        ("can_last_rx_ms", C.c_uint32),
        ("can_ever_rx", C.c_bool),
        ("manual_inhibit", C.c_bool),
        ("actuator_supply_good", C.c_bool),
        ("service_request", C.c_bool),
        ("service_exit", C.c_bool),
        ("fault_reset_request", C.c_bool),
        ("channel", ChannelFeedback * MAX_CHANNELS),
    ]


class Output(C.Structure):
    _fields_ = [
        ("timestamp_ms", C.c_uint32),
        ("state", C.c_int),
        ("previous_state", C.c_int),
        ("reason", C.c_int),
        ("target", C.c_int),
        ("command_enabled", C.c_bool),
        ("channel_command_deg", C.c_float * MAX_CHANNELS),
        ("channel_status", C.c_int * MAX_CHANNELS),
        ("output_confirmed", C.c_bool),
        ("inhibit_flags", C.c_uint32),
        ("active_faults", C.c_uint32),
        ("latched_faults", C.c_uint32),
        ("signal_status", C.c_uint8 * NSIG),
        ("transition_count", C.c_uint32),
        ("state_entered_ms", C.c_uint32),
    ]


def _find_library() -> Path:
    root = Path(__file__).resolve().parents[2]
    env = os.environ.get("AERO_CORE_LIB")
    if env:
        return Path(env)
    ext = {"darwin": "dylib", "win32": "dll"}.get(sys.platform, "so")
    cand = root / "build" / f"aero_core.{ext}"
    if cand.exists():
        return cand
    raise FileNotFoundError("C core library not built. Run `python aero.py setup` from the repository root.")


def _load():
    lib = C.CDLL(str(_find_library()))
    for name, restype in (
        ("aero_sizeof_config", C.c_size_t),
        ("aero_sizeof_inputs", C.c_size_t),
        ("aero_sizeof_output", C.c_size_t),
        ("aero_sizeof_supervisor", C.c_size_t),
        ("aero_config_is_complete", C.c_bool),
    ):
        getattr(lib, name).restype = restype
    lib.aero_config_init_tbd.argtypes = [C.POINTER(Config)]
    lib.aero_config_is_complete.argtypes = [C.POINTER(Config)]
    lib.aero_init.argtypes = [C.c_void_p, C.POINTER(Config), C.c_uint32]
    lib.aero_tick.argtypes = [C.c_void_p, C.POINTER(Inputs), C.c_uint32]
    lib.aero_tick.restype = C.POINTER(Output)
    lib.aero_reason_name.argtypes = [C.c_int]
    lib.aero_reason_name.restype = C.c_char_p

    for py, c_fn in ((Config, "aero_sizeof_config"), (Inputs, "aero_sizeof_inputs"), (Output, "aero_sizeof_output")):
        if C.sizeof(py) != getattr(lib, c_fn)():
            raise RuntimeError(f"{py.__name__} layout mismatch with C core; update core.py to match aero_core.h")
    return lib


LIB = _load()
REASONS: tuple[str, ...] = tuple(
    LIB.aero_reason_name(i).decode() for i in range(64) if LIB.aero_reason_name(i).decode() != "?"
)


def _f(v) -> float:
    return math.nan if v is None else float(v)


def _ms(v) -> int:
    return TBD_MS if v is None else int(v)


def config_from_dict(d: dict) -> Config:
    """Build a C config from a JSON-style dict. Missing / null values stay TBD."""
    cfg = Config()
    LIB.aero_config_init_tbd(C.byref(cfg))
    for i, name in enumerate(SIGNALS):
        s = d.get("signals", {}).get(name, {})
        cfg.signal_required[i] = bool(s.get("required", True))
        cfg.signal_timeout_ms[i] = _ms(s.get("timeout_ms"))
        cfg.signal_min[i] = _f(s.get("min"))
        cfg.signal_max[i] = _f(s.get("max"))
    for key in ("can_bus_timeout_ms", "inputs_stable_ms", "fault_recovery_stable_ms",
                "ld_entry_debounce_ms", "motion_timeout_ms"):
        setattr(cfg, key, _ms(d.get(key)))
    for key in ("ld_entry_speed_mps", "ld_exit_speed_mps", "ld_entry_throttle", "ld_exit_throttle",
                "hdf_brake_bar", "hdf_steering_rad", "hdf_yaw_rate_rps", "hdf_lat_accel_mps2",
                "position_tolerance_deg"):
        setattr(cfg, key, _f(d.get(key)))
    ch = d.get("channels", {})
    cfg.channel_count = int(ch.get("count") or 0)
    for i in range(MAX_CHANNELS):
        hdf = ch.get("hdf_position_deg") or []
        ld = ch.get("ld_position_deg") or []
        cfg.hdf_position_deg[i] = _f(hdf[i]) if i < len(hdf) else math.nan
        cfg.ld_position_deg[i] = _f(ld[i]) if i < len(ld) else math.nan
    cfg.variable_aero_enabled = bool(d.get("variable_aero_enabled", False))
    return cfg


def config_is_complete(cfg: Config) -> bool:
    return bool(LIB.aero_config_is_complete(C.byref(cfg)))


class Supervisor:
    """Owns one C aero_supervisor_t instance."""

    def __init__(self, cfg: Config, now_ms: int = 0):
        n = LIB.aero_sizeof_supervisor()
        self._buf = (C.c_uint64 * ((n + 7) // 8))()  # 8-byte aligned opaque storage
        self.cfg = cfg
        LIB.aero_init(C.byref(self._buf), C.byref(cfg), now_ms)

    def tick(self, inputs: Inputs, now_ms: int) -> Output:
        out = LIB.aero_tick(C.byref(self._buf), C.byref(inputs), now_ms).contents
        snapshot = Output()
        C.memmove(C.byref(snapshot), C.byref(out), C.sizeof(Output))
        return snapshot
