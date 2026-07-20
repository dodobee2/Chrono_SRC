"""Builds a rigid-terrain Chrono scenario (system + terrain + optional obstacle + rover).

Reuses terrain_factory.build_rigid_flat_terrain (flat conditions only) and
rover_factory.build_rover_from_spec unmodified. Two things this pilot needs
that terrain_factory deliberately does NOT support are handled locally, here,
instead of extending the shared factory (keeps terrain_factory.py untouched
so it doesn't collide with other work on the same file):

  - slope: the floor itself is tilted (see _build_tilted_floor), world
    gravity stays standard vertical. TerrainScenario.slope_long_deg is kept
    at 0 for the same reason build_rigid_flat_terrain is only called for
    flat conditions -- it refuses any slope.
  - obstacle: a single fixed box is added directly with plain pychrono calls after the
    factory-built floor, following the pattern in handoff/map.py::create_fixed_box.
    TerrainScenario.obstacles is kept empty for the same reason (build_rigid_flat_terrain
    raises NotImplementedError otherwise). Not combined with slope in any
    current CONDITIONS entry (see presets.py) -- _add_obstacle assumes a flat floor.

ChSystemNSC is used, not SMC -- see src/chrono/system_factory.py for why this
module routes system/material creation through make_nsc_system() and
make_nsc_contact_material() instead of calling chrono.ChSystemNSC()/
ChContactMaterialNSC() directly.

Slope implementation history (2026-07-15): this originally tilted *gravity*
instead of the floor (kept world-vertical geometry, rotated the gravity
vector by slope_deg -- same trick used in src/experiments/scm_pilot). That
had two problems, discovered in this order:
  1. The tilt's sign was backwards (fixed once, then found to still be wrong
     in spirit -- see (2)).
  2. Independent of sign or commanded torque, a gravity vector with a large
     horizontal component made the rover pitch over and flip during the
     zero-torque *settle* phase alone, at 10-15 deg and eventually even 5 deg
     -- confirmed by running with torque_fraction=0.0 and watching it flip
     anyway. This is a real numerical instability in how this rigid-body
     setup responds to non-vertical gravity, not a torque-tuning problem;
     reducing torque_fraction from 0.6 down to 0.15 changed the outcome
     essentially not at all (distance stayed ~51-52m either way for
     slope_15deg). Static tip-over geometry (atan((wheelbase/2)/cg_height))
     is nowhere near these angles (~59 deg for main_v01), so this was not a
     real physical tip-over -- switching to an actual tilted floor (standard
     vertical gravity, physically correct incline contact geometry) avoids
     the whole failure mode rather than trying to tune around it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

from ...integration_schemas import RoverSpec, TerrainGeometrySpec, TerrainMaterialSpec, TerrainScenario
from ...registries import RoverRegistry, TerrainMaterialRegistry, TerrainRegistry
from ...chrono.rover_factory import build_rover_from_spec
from ...chrono.system_factory import make_nsc_contact_material, make_nsc_system
from ...chrono.terrain_factory import (
    DEFAULT_RESTITUTION,
    DEFAULT_RIGID_FRICTION,
    build_rigid_flat_terrain,
    build_terrain_from_scenario,
)
from .presets import (
    FRICTION_MATERIAL_IDS,
    JONGMIN_ARENA_CONDITION,
    JONGMIN_ARENA_CONDITION_ID,
    JONGMIN_ARENA_TERRAIN_ID,
    OBSTACLE_PRESETS,
    PATCH_DIMENSIONS_XYZ_M,
    ROVER_IDS,
    RigidCondition,
)

# Reused verbatim from src/chrono/irrlicht_jongmin_arena_viewer.py's
# run_viewer -- that spawn point is the one this arena has actually been
# driven from before (via the Irrlicht viewer), so it's not a new guess.
JONGMIN_ARENA_SPAWN_XY_M = (-2.50, 0.0)

if TYPE_CHECKING:
    from ...chrono.vendor.rover_module_v01.rover_builder import RoverInstance

PROJECT_ROOT = Path(__file__).resolve().parents[3]

SPAWN_DROP_M = 0.05
SPAWN_MARGIN_M = 0.3


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


def _load_jongmin_arena_terrain() -> tuple[TerrainScenario, TerrainMaterialSpec | None]:
    terrain = TerrainRegistry(PROJECT_ROOT / "terrain_scenarios", repo_root=PROJECT_ROOT).load(JONGMIN_ARENA_TERRAIN_ID)
    material_registry = TerrainMaterialRegistry(PROJECT_ROOT / "terrain_materials", repo_root=PROJECT_ROOT)
    material = material_registry.load(terrain.material_id) if terrain.material_id in material_registry.ids() else None
    return terrain, material


def terrain_context_for(condition: RigidCondition) -> TerrainContext:
    """Builds the TerrainContext for a condition without touching pychrono."""
    if condition.condition_id == JONGMIN_ARENA_CONDITION_ID:
        terrain, material = _load_jongmin_arena_terrain()
        return TerrainContext(
            slope_deg=terrain.slope_long_deg,
            friction_nominal=material.friction_nominal if material and material.friction_nominal is not None else 0.6,
            rolling_resistance_nominal=material.rolling_resistance_nominal
            if material and material.rolling_resistance_nominal is not None
            else 0.05,
            # Multi-zone arena, not a single fixed obstacle -- this pilot's
            # obstacle_height_m/obstacle_distance_m concept doesn't apply;
            # terrain_only's predictor already notes when obstacle modeling
            # is skipped.
            obstacle_height_m=0.0,
            obstacle_distance_m=0.0,
        )
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


def _incline_rotation(chrono: Any, slope_deg: float) -> Any:
    """Rotation tilting local +X (forward/uphill) upward in world Z by slope_deg.

    Positive slope_deg means climbing resists forward motion once gravity is
    projected onto the incline plane, matching required_traction_force_n's
    F_req = mg*sin(slope) + ... assumption -- verified empirically (not just
    assumed) after implementing this: distance traveled decreases as
    slope_deg increases under a fixed torque command (see scenario.py commit
    message / test output referenced 2026-07-15), the opposite of the
    gravity-tilt bug this replaced.
    """
    return chrono.QuatFromAngleY(-math.radians(slope_deg))


def _build_tilted_floor(system: Any, chrono: Any, slope_deg: float, material: TerrainMaterialSpec) -> Any:
    """Builds a rigid floor tilted by slope_deg. World gravity stays standard vertical.

    The floor's top surface passes through the world origin, at the same
    rotation returned by _incline_rotation -- build_pilot_scenario's spawn
    frame uses the identical rotation so the rover starts flush with the
    incline instead of dropping onto it at the wrong angle.
    """
    length_x, width_y, thickness = PATCH_DIMENSIONS_XYZ_M
    friction = material.friction_nominal if material and material.friction_nominal is not None else DEFAULT_RIGID_FRICTION
    restitution = material.restitution if material and material.restitution is not None else DEFAULT_RESTITUTION

    floor_material = make_nsc_contact_material(friction=friction, restitution=restitution)
    floor = chrono.ChBodyEasyBox(length_x, width_y, thickness, 2000.0, True, True, floor_material)
    floor.SetName(f"rigid_pilot_incline_{slope_deg:g}deg_floor")

    rotation = _incline_rotation(chrono, slope_deg)
    local_top_center = chrono.ChVector3d(0.0, 0.0, thickness / 2.0)
    floor.SetPos(-rotation.Rotate(local_top_center))
    floor.SetRot(rotation)
    floor.SetFixed(True)
    floor.EnableCollision(True)
    system.Add(floor)
    return floor


def _add_obstacle(system: Any, chrono: Any, condition: RigidCondition, patch_length_m: float) -> None:
    if condition.obstacle_key is None:
        return
    preset = OBSTACLE_PRESETS[condition.obstacle_key]
    material = make_nsc_contact_material(friction=0.8, restitution=0.02)
    box = chrono.ChBodyEasyBox(0.05, preset.width_m, preset.height_m, 2000.0, True, True, material)
    box.SetName(f"obstacle_{condition.obstacle_key}")
    start_x = -patch_length_m / 2.0 + SPAWN_MARGIN_M
    box.SetPos(chrono.ChVector3d(start_x + preset.distance_from_start_m, 0.0, preset.height_m / 2.0))
    box.SetFixed(True)
    box.EnableCollision(True)
    system.Add(box)


def _build_jongmin_arena_scenario(rover_key: str, torque_fraction: float, chrono: Any) -> PilotScenario:
    """Builds scout/main on Jongmin's real 5-zone arena instead of this pilot's own flat/tilted floor.

    Reuses terrain_factory.build_terrain_from_scenario + the arena's own
    code_factory builder (terrain_scenarios/jongmin_arena_v01/chrono_factory.py)
    unmodified -- that adapter already remaps SMC->NSC contact materials and
    skips the SCM zone that needs pychrono.vehicle, so this only needs
    pychrono core like every other condition in this pilot. Building this
    arena (mesh + procedural rock zones) is noticeably slower than the other
    conditions -- callers should use a longer per-attempt timeout.
    """
    rover_spec = _torque_limited_rover(load_rover_spec(rover_key), torque_fraction)
    terrain, material = _load_jongmin_arena_terrain()
    terrain_context = terrain_context_for(JONGMIN_ARENA_CONDITION)

    system = make_nsc_system()
    build_terrain_from_scenario(system, terrain, material)

    spawn_x, spawn_y = JONGMIN_ARENA_SPAWN_XY_M
    spawn_z = max(0.08, rover_spec.wheel_radius_m + 0.03)
    spawn_frame = chrono.ChFramed(chrono.ChVector3d(spawn_x, spawn_y, spawn_z), chrono.QUNIT)
    rover = build_rover_from_spec(system, rover_spec, spawn_frame=spawn_frame)

    return PilotScenario(
        system=system,
        rover=rover,
        rover_spec=rover_spec,
        terrain_material=material,
        terrain_context=terrain_context,
    )


def build_pilot_scenario(rover_key: str, condition: RigidCondition, torque_fraction: float) -> PilotScenario:
    """Build one (rover, condition) rigid pilot scenario. Only needs pychrono core."""
    import pychrono as chrono

    if condition.condition_id == JONGMIN_ARENA_CONDITION_ID:
        return _build_jongmin_arena_scenario(rover_key, torque_fraction, chrono)

    rover_spec = _torque_limited_rover(load_rover_spec(rover_key), torque_fraction)
    material = load_friction_material(condition.friction_key)
    terrain_context = terrain_context_for(condition)

    system = make_nsc_system()

    start_x = -PATCH_DIMENSIONS_XYZ_M[0] / 2.0 + SPAWN_MARGIN_M
    if abs(condition.slope_deg) > 1e-9:
        _build_tilted_floor(system, chrono, condition.slope_deg, material)
        rotation = _incline_rotation(chrono, condition.slope_deg)
        spawn_local = chrono.ChVector3d(start_x, 0.0, SPAWN_DROP_M)
        spawn_frame = chrono.ChFramed(rotation.Rotate(spawn_local), rotation)
    else:
        terrain_scenario = _make_flat_terrain_scenario(f"rigid_pilot_{condition.condition_id}", material.material_id)
        build_rigid_flat_terrain(system, terrain_scenario, material)
        _add_obstacle(system, chrono, condition, PATCH_DIMENSIONS_XYZ_M[0])
        spawn_frame = chrono.ChFramed(chrono.ChVector3d(start_x, 0.0, SPAWN_DROP_M), chrono.QUNIT)

    rover = build_rover_from_spec(system, rover_spec, spawn_frame=spawn_frame)

    return PilotScenario(
        system=system,
        rover=rover,
        rover_spec=rover_spec,
        terrain_material=material,
        terrain_context=terrain_context,
    )
