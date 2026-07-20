"""Presets for the rigid-terrain Scout->Main transfer pilot.

One variable is perturbed at a time from a flat/mid-friction baseline
(9 conditions total), not a full factorial sweep -- keeps run count small
for a first-pass pilot. Extend CONDITIONS here first if a fuller sweep is
needed later; do not fork a second condition table elsewhere.
"""

from __future__ import annotations

from dataclasses import dataclass

ROVER_IDS: dict[str, str] = {
    "scout": "scout_v01",
    "main": "main_v01",
}

# terrain_materials/<id>.yaml -- all model_type: rigid.
FRICTION_MATERIAL_IDS: dict[str, str] = {
    "low": "rigid_low_friction_v0",
    "mid": "rigid_mid_friction_v0",
    "high": "rigid_high_friction_v0",
}

BASELINE_FRICTION_KEY = "mid"
# Long enough that main_v01 (reaches ~0.6 m/s) can't drive off the far edge
# within DEFAULT_COMMAND.duration_s. The original 4.0m was too short: main
# reached x=+2.0 (the patch edge) at t=6.25s of a 6.5s run and pitched off
# the edge, which looked exactly like a rollover/instability bug until the
# trajectory was inspected (2026-07-15) -- not a torque or incline problem.
PATCH_DIMENSIONS_XYZ_M: tuple[float, float, float] = (12.0, 1.5, 0.3)


@dataclass(frozen=True)
class ObstaclePreset:
    height_m: float
    width_m: float
    distance_from_start_m: float


OBSTACLE_PRESETS: dict[str, ObstaclePreset] = {
    "low": ObstaclePreset(height_m=0.02, width_m=0.4, distance_from_start_m=1.2),
    "high": ObstaclePreset(height_m=0.05, width_m=0.4, distance_from_start_m=1.2),
}


@dataclass(frozen=True)
class RigidCondition:
    """One (slope, friction, obstacle) test condition."""

    condition_id: str
    slope_deg: float
    friction_key: str
    obstacle_key: str | None


CONDITIONS: tuple[RigidCondition, ...] = (
    RigidCondition("flat", 0.0, BASELINE_FRICTION_KEY, None),
    RigidCondition("slope_5deg", 5.0, BASELINE_FRICTION_KEY, None),
    RigidCondition("friction_low", 0.0, "low", None),
    RigidCondition("friction_mid", 0.0, "mid", None),
    RigidCondition("friction_high", 0.0, "high", None),
    RigidCondition("obstacle_low", 0.0, BASELINE_FRICTION_KEY, "low"),
    RigidCondition("obstacle_high", 0.0, BASELINE_FRICTION_KEY, "high"),
)

# Excluded from CONDITIONS (2026-07-15): main_v01 reliably flips over at
# 10-15 deg under the torque-limited command. Three separate, real issues
# were found and fixed along the way before landing on this conclusion --
# recorded here so nobody re-adds these expecting a quick win:
#   1. Incline geometry: the floor was tilted via a gravity-vector hack, not
#      an actual tilted body (world gravity stayed vertical). That alone
#      caused a violent flip even at rest (zero commanded torque) at
#      10-15 deg. Fixed by tilting the floor itself instead -- see
#      scenario.py's module docstring.
#   2. Floor length: PATCH_DIMENSIONS_XYZ_M was 4.0m, too short for main_v01
#      to finish a 6.5s run without driving off the far edge (reached it at
#      t=6.25s and pitched off). Bumped to 12.0m. This explained why flat and
#      the friction conditions also looked unstable, and made slope_15deg's
#      *final frame* look fine too.
#   3. The completion check itself only looked at the final trajectory frame.
#      After fix (2), slope_15deg's summary showed completed=True with a
#      plausible final_pitch/final_roll -- but its mean_pitch_deg was 83 deg,
#      which should never happen for a rover actually driving. Inspecting the
#      trajectory showed it rolled to ~179 deg at t=0.65s and was still
#      tumbling at t=6.5s; the final frame just happened to pass through an
#      in-tolerance angle mid-rotation. summarize() now checks max abs
#      pitch/roll over the whole run (RunSummary.max_pitch_deg/max_roll_deg),
#      which correctly reports slope_15deg as completed=False
#      (max_roll_deg=178.97).
# slope_5deg is clean throughout (max_roll_deg stays under 6 deg, settles at
# pitch=-5.00 matching the incline angle) so the incline geometry itself is
# verified correct; only 10/15 deg actually flip. Root cause of *why* it
# flips at these specific angles (not a monotonic function of torque_fraction
# swept 0.6->0.2) was not found. Do not silently re-add either without
# checking mean_slip/max_pitch_deg/max_roll_deg sanity first.
UNSTABLE_CONDITIONS: tuple[RigidCondition, ...] = (
    RigidCondition("slope_10deg", 10.0, BASELINE_FRICTION_KEY, None),
    RigidCondition("slope_15deg", 15.0, BASELINE_FRICTION_KEY, None),
)

# Jongmin's real 5-zone arena (terrain_scenarios/jongmin_arena_v01), added
# 2026-07-16 as an extra selectable condition in app.py's 3-B live panel only
# -- NOT part of CONDITIONS, so scripts/run_rigid_transfer_pilot.py's 7-way
# CLI batch and its tests are untouched. slope_deg/friction_key/obstacle_key
# below are unused placeholders for this entry: scenario.py's
# build_pilot_scenario and terrain_context_for special-case
# condition_id == JONGMIN_ARENA_CONDITION_ID and load the arena's real
# TerrainScenario/TerrainMaterialSpec instead of building from these fields.
# This arena is slower to build than the other conditions (multi-zone mesh +
# rock generation) -- callers should use a longer per-attempt timeout (see
# app.py's render_rigid_transfer_live_run).
JONGMIN_ARENA_CONDITION_ID = "jongmin_arena"
JONGMIN_ARENA_TERRAIN_ID = "jongmin_arena_v01"
JONGMIN_ARENA_CONDITION = RigidCondition(JONGMIN_ARENA_CONDITION_ID, 0.0, BASELINE_FRICTION_KEY, None)
EXTRA_CONDITIONS: tuple[RigidCondition, ...] = (JONGMIN_ARENA_CONDITION,)


@dataclass(frozen=True)
class TorqueCommand:
    """Torque-limited straight command -- wheel_torque mode only, no ideal wheel_speed motor.

    Overrides each rover's command_type to "wheel_torque" via RoverSpec.metadata
    before building (see scenario.py::_torque_limited_rover). scout_v01/main_v01
    are registered with command_type: wheel_speed (Hojin's own verified default),
    so this is a deliberate pilot-only deviation, not a change to the registered spec.
    """

    torque_fraction: float = 0.6  # fraction of max_wheel_torque_nm commanded
    duration_s: float = 6.0
    settle_s: float = 0.5
    ramp_s: float = 0.5
    timestep_s: float = 2.0e-3
    log_period_s: float = 0.05


DEFAULT_COMMAND = TorqueCommand()
