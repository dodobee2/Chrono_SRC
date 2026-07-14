"""Fixed presets for the scout-to-main SCM pilot: slopes, soils, rovers, command.

Kept intentionally small per the pilot scope: flat/10deg slope, loose/medium/
firm soil, scout_v01/main_v01, one straight cruise command. Extend here
first if the pilot needs a wider sweep -- do not fork a second preset table
elsewhere.
"""

from __future__ import annotations

from dataclasses import dataclass

SLOPE_PRESETS_DEG: dict[str, float] = {
    "flat": 0.0,
    "slope_10deg": 10.0,
}

# terrain_materials/<id>.yaml -- all model_type: scm.
SOIL_PRESET_MATERIAL_IDS: dict[str, str] = {
    "loose": "loose_sand_scm_v0",
    "medium": "medium_soil_scm_v0",
    "firm": "firm_soil_scm_v0",
}

# rover_models/<id>/rover.yaml -- both Chrono-verified (see handoff/rover_module_v01).
ROVER_IDS: dict[str, str] = {
    "scout": "scout_v01",
    "main": "main_v01",
}

PATCH_DIMENSIONS_XYZ_M: tuple[float, float, float] = (2.0, 1.0, 0.3)


@dataclass(frozen=True)
class CruiseCommand:
    """A single straight-line command, ramped in over ramp_s then held.

    Fraction of max is applied in whichever mode the rover's own
    command_type is (see rover.yaml metadata) -- mirrors
    handoff/rover_module_v01/code/scripts/run_rover_check.py::cruise_command
    rather than forcing torque mode on a rover verified in speed mode.
    """

    duration_s: float = 6.0
    settle_s: float = 1.0
    ramp_s: float = 1.0
    wheel_speed_fraction: float = 0.6
    wheel_torque_fraction: float = 0.5
    timestep_s: float = 2.0e-3
    log_period_s: float = 0.05


DEFAULT_COMMAND = CruiseCommand()
