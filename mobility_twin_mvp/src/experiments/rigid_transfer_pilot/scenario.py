"""Builds a rigid-terrain Chrono scenario (system + terrain + optional obstacle + rover).

Reuses terrain_factory.build_rigid_flat_terrain and rover_factory.build_rover_from_spec
unmodified. Two things this pilot needs that terrain_factory deliberately does NOT
support are handled locally, here, instead of extending the shared factory (keeps
terrain_factory.py untouched so it doesn't collide with other work on the same file):

  - slope: applied by tilting gravity (same technique as
    src/experiments/scm_pilot/scenario.py -- see its docstring for the documented
    limitation: straight-line longitudinal response only, no lateral tip-over or
    approach geometry). The terrain patch itself is always built flat
    (slope_long_deg=0), which build_rigid_flat_terrain requires.
  - obstacle: a single fixed box is added directly with plain pychrono calls after the
    factory-built floor, following the pattern in handoff/map.py::create_fixed_box.
    TerrainScenario.obstacles is kept empty for the same reason (build_rigid_flat_terrain
    raises NotImplementedError otherwise).

ChSystemNSC is used, not SMC: src/chrono/vendor/rover_module_v01/rover_builder.py
hardcodes ChContactMaterialNSC for the rover's own wheels/chassis, and everything
verified working in this project so far (rover_factory tests, smoke_scenario.py) uses
NSC. Mixing NSC rover contact materials into a ChSystemSMC has not been attempted and is
a likely source of a hard Chrono error, not just an untested combination -- see the
pilot review this module implements the fixes for.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

from ...integration_schemas import RoverSpec, TerrainGeometrySpec, TerrainMaterialSpec, TerrainScenario
from ...registries import RoverRegistry, TerrainMaterialRegistry
from ...chrono.rover_factory import build_rover_from_spec
from ...chrono.terrain_factory import build_rigid_flat_terrain
from .presets import FRICTION_MATERIAL_IDS, OBSTACLE_PRESETS, PATCH_DIMENSIONS_XYZ_M, ROVER_IDS, RigidCondition

if TYPE_CHECKING:
    from ...chrono.vendor.rover_module_v01.rover_builder import RoverInstance

PROJECT_ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class TerrainContext:
    """Everything a predictor might need to know about the condition's terrain.

    Pure data, no pychrono dependency -- safe to construct without building a
    live scenario (the CLI script builds one per condition to hand to
    predictors without re-running the simulation).
    """

    slope_deg: float
    friction_nominal: float
    rolling_resistance_nominal: float
    obstacle_height_m: float
    obstacle_distance_m: float


@dataclass(frozen=True)
class PilotScenario:
    system: Any
    rover: RoverInstance
    rover_spec: RoverSpec
    terrain_material: TerrainMaterialSpec
    terrain_context: TerrainContext


def load_rover_spec(rover_key: str) -> RoverSpec:
    if rover_key not in ROVER_IDS:
        raise ValueError(f"unknown rover_key {rover_key!r}; expected one of {sorted(ROVER_IDS)}")
    return RoverRegistry(PROJECT_ROOT / "rover_models", repo_root=PROJECT_ROOT).load(ROVER_IDS[rover_key])


def load_friction_material(friction_key: str) -> TerrainMaterialSpec:
    if friction_key not in FRICTION_MATERIAL_IDS:
        raise ValueError(f"unknown friction_key {friction_key!r}; expected one of {sorted(FRICTION_MATERIAL_IDS)}")
    return TerrainMaterialRegistry(PROJECT_ROOT / "terrain_materials", repo_root=PROJECT_ROOT).load(
        FRICTION_MATERIAL_IDS[friction_key]
    )


def terrain_context_for(condition: RigidCondition) -> TerrainContext:
    """Builds the TerrainContext for a condition without touching pychrono."""
    material = load_friction_material(condition.friction_key)
    obstacle_preset = OBSTACLE_PRESETS.get(condition.obstacle_key) if condition.obstacle_key else None
    return TerrainContext(
        slope_deg=condition.slope_deg,
        friction_nominal=material.friction_nominal if material.friction_nominal is not None else 0.6,
        rolling_resistance_nominal=material.rolling_resistance_nominal
        if material.rolling_resistance_nominal is not None
        else 0.05,
        obstacle_height_m=obstacle_preset.height_m if obstacle_preset else 0.0,
        obstacle_distance_m=obstacle_preset.distance_from_start_m if obstacle_preset else 0.0,
    )


def _torque_limited_rover(rover: RoverSpec, torque_fraction: float) -> RoverSpec:
    """Overrides command_type to wheel_torque via metadata (no ideal wheel_speed motor,
    per pilot spec). scout_v01/main_v01 are registered with command_type: wheel_speed
    (Hojin's verified default) -- this is a deliberate pilot-only deviation, not a change
    to the registered spec.
    """
    metadata = dict(rover.metadata)
    metadata["command_type"] = "wheel_torque"
    metadata["pilot_note"] = (
        f"command_type overridden to wheel_torque for rigid_transfer_pilot (torque_fraction={torque_fraction})"
    )
    return replace(rover, metadata=metadata)


def _make_flat_terrain_scenario(terrain_id: str, material_id: str) -> TerrainScenario:
    return TerrainScenario(
        terrain_id=terrain_id,
        display_name=f"Rigid transfer pilot patch ({terrain_id})",
        terrain_type="rigid",
        surface_hint="rigid_transfer_pilot",
        geometry=TerrainGeometrySpec.from_mapping({"frame_id": terrain_id}),
        material_id=material_id,
        dimensions_xyz_m=PATCH_DIMENSIONS_XYZ_M,
        frame_id=terrain_id,
        random_seed=1,
    )


def _tilted_gravity(chrono: Any, slope_deg: float, magnitude_mps2: float = 9.81) -> Any:
    import math

    theta = math.radians(slope_deg)
    return chrono.ChVector3d(magnitude_mps2 * math.sin(theta), 0.0, -magnitude_mps2 * math.cos(theta))


def _add_obstacle(system: Any, chrono: Any, condition: RigidCondition, patch_length_m: float) -> None:
    if condition.obstacle_key is None:
        return
    preset = OBSTACLE_PRESETS[condition.obstacle_key]
    material = chrono.ChContactMaterialNSC()
    material.SetFriction(0.8)
    material.SetRestitution(0.02)
    box = chrono.ChBodyEasyBox(0.05, preset.width_m, preset.height_m, 2000.0, True, True, material)
    box.SetName(f"obstacle_{condition.obstacle_key}")
    start_x = -patch_length_m / 2.0 + 0.3
    box.SetPos(chrono.ChVector3d(start_x + preset.distance_from_start_m, 0.0, preset.height_m / 2.0))
    box.SetFixed(True)
    box.EnableCollision(True)
    system.Add(box)


def build_pilot_scenario(rover_key: str, condition: RigidCondition, torque_fraction: float) -> PilotScenario:
    """Build one (rover, condition) rigid pilot scenario. Only needs pychrono core."""
    import pychrono as chrono

    rover_spec = _torque_limited_rover(load_rover_spec(rover_key), torque_fraction)
    material = load_friction_material(condition.friction_key)
    terrain_context = terrain_context_for(condition)

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(_tilted_gravity(chrono, condition.slope_deg))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    terrain_scenario = _make_flat_terrain_scenario(f"rigid_pilot_{condition.condition_id}", material.material_id)
    build_rigid_flat_terrain(system, terrain_scenario, material)
    _add_obstacle(system, chrono, condition, PATCH_DIMENSIONS_XYZ_M[0])

    spawn_z = 0.05
    start_x = -PATCH_DIMENSIONS_XYZ_M[0] / 2.0 + 0.3
    spawn_frame = chrono.ChFramed(chrono.ChVector3d(start_x, 0.0, spawn_z), chrono.QUNIT)
    rover = build_rover_from_spec(system, rover_spec, spawn_frame=spawn_frame)

    return PilotScenario(
        system=system,
        rover=rover,
        rover_spec=rover_spec,
        terrain_material=material,
        terrain_context=terrain_context,
    )
