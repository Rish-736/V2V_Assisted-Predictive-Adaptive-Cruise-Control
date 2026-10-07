"""
v2vacc -- V2V-Assisted Predictive Adaptive Cruise Control
BECE302L Control Systems, VIT Vellore.

Layout
------
    plant.py        motor + gap-kinematics models (the twin's physics)
    controller.py   Python mirror of the firmware control laws
    twin.py         digital twin: open-loop (live) and closed-loop (offline)
    telemetry.py    parser for the serial telemetry format
    serial_link.py  threaded serial reader + log replay

Quick start, no hardware required:

    python python/scripts/sim_only.py --compare
    python python/scripts/dashboard.py --replay data/example_run.csv
"""

from .plant import PlantParams, FirstOrderPlant, SecondOrderPlant, VehicleKinematics
from .controller import (Mode, LinkState, Gains, GapPolicy, BrakePolicy, PID,
                         FollowerController)
from .twin import OpenLoopTwin, ClosedLoopTwin, compare, FitMetrics, lead_scenario
from .telemetry import FollowerRecord, LeadRecord, parse_line, parse_file
from .serial_link import SerialTelemetry, ReplaySource, list_ports, autodetect_port

__version__ = "0.3.0"

__all__ = [
    "PlantParams", "FirstOrderPlant", "SecondOrderPlant", "VehicleKinematics",
    "Mode", "LinkState", "Gains", "GapPolicy", "BrakePolicy", "PID",
    "FollowerController", "OpenLoopTwin", "ClosedLoopTwin", "compare",
    "FitMetrics", "lead_scenario", "FollowerRecord", "LeadRecord",
    "parse_line", "parse_file", "SerialTelemetry", "ReplaySource",
    "list_ports", "autodetect_port", "__version__",
]
