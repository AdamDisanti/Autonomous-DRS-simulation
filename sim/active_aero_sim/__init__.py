"""Software-in-the-loop simulator for the active aero supervisor.

SIMULATION ONLY. Synthetic thresholds and actuator physics; not vehicle-ready.
All supervisor decisions come from the portable C core via ctypes.
"""
from .sim import Sim

__all__ = ["Sim"]
