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
PATCH_DIMENSIONS_XYZ_M: tuple[float, float, float] = (4.0, 1.5, 0.3)


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
    RigidCondition("slope_10deg", 10.0, BASELINE_FRICTION_KEY, None),
    RigidCondition("slope_15deg", 15.0, BASELINE_FRICTION_KEY, None),
    RigidCondition("friction_low", 0.0, "low", None),
    RigidCondition("friction_mid", 0.0, "mid", None),
    RigidCondition("friction_high", 0.0, "high", None),
    RigidCondition("obstacle_low", 0.0, BASELINE_FRICTION_KEY, "low"),
    RigidCondition("obstacle_high", 0.0, BASELINE_FRICTION_KEY, "high"),
)


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
