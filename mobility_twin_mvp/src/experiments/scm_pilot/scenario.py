"""Builds a Chrono system + SCM patch + rover for one pilot run.

Slope is applied by tilting gravity, not the terrain mesh: for a straight
cruise on a single flat SCM patch, tilting gravity by slope_deg about the Y
axis is physically equivalent to driving up an incline (same
mg*sin/mg*cos decomposition the heuristic backend already uses in
src/mobility_physics.py::required_traction_force_n) and avoids building a
sloped SCM mesh. This does NOT reproduce a real sloped approach/departure or
lateral tip-over geometry -- it is only valid for straight longitudinal
slope response (slip/torque/energy), which is exactly what this pilot
measures. Because the mesh itself stays flat at z=0, sinkage can be read
straight off wheel height (see runner.py) without needing to query SCMTerrain
deformation through an unconfirmed API. If lateral slope or approach
geometry is ever needed, extend terrain_factory instead of stretching this
trick further.

UNVERIFIED: pychrono.vehicle (and therefore SCMTerrain) currently fails to
import in this project's `chrono` conda env (DLL init error -- and has been
observed to hang rather than fail cleanly when attempted in a process that
already built other Chrono objects, see docs/ENVIRONMENT_SETUP.md), so none
of this module has been run end-to-end. In particular, whether SCMTerrain
works correctly under ChSystemNSC has not been checked --
src/chrono/vendor/rover_module_v01/rover_builder.py hardcodes
ChContactMaterialNSC for the rover's own wheels/chassis, so this scenario
defaults to ChSystemNSC to stay consistent with that, rather than the
ChSystemSMC used by handoff/map.py's rigid-body arena. Confirm this
combination actually initializes once pychrono.vehicle is fixed; switch to
ChSystemSMC (and matching contact materials in the vendored rover builder)
if SCMTerrain turns out to require it.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

from ...integration_schemas import RoverSpec, TerrainGeometrySpec, TerrainMaterialSpec, TerrainScenario
from ...registries import RoverRegistry, TerrainMaterialRegistry
from ...chrono.rover_factory import build_rover_from_spec
from ...chrono.terrain_factory import build_scm_terrain
from .presets import PATCH_DIMENSIONS_XYZ_M, ROVER_IDS, SOIL_PRESET_MATERIAL_IDS

if TYPE_CHECKING:
    from ...chrono.vendor.rover_module_v01.rover_builder import RoverInstance

PROJECT_ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class PilotScenario:
    system: Any
    terrain: Any
    rover: RoverInstance
    rover_spec: RoverSpec
    soil_material: TerrainMaterialSpec
    slope_deg: float


def load_rover_spec(rover_key: str) -> RoverSpec:
    if rover_key not in ROVER_IDS:
        raise ValueError(f"unknown rover_key {rover_key!r}; expected one of {sorted(ROVER_IDS)}")
    return RoverRegistry(PROJECT_ROOT / "rover_models", repo_root=PROJECT_ROOT).load(ROVER_IDS[rover_key])


def load_soil_material(soil_key: str) -> TerrainMaterialSpec:
    if soil_key not in SOIL_PRESET_MATERIAL_IDS:
        raise ValueError(f"unknown soil_key {soil_key!r}; expected one of {sorted(SOIL_PRESET_MATERIAL_IDS)}")
    return TerrainMaterialRegistry(PROJECT_ROOT / "terrain_materials", repo_root=PROJECT_ROOT).load(
        SOIL_PRESET_MATERIAL_IDS[soil_key]
    )


def _make_flat_terrain_scenario(terrain_id: str, material_id: str) -> TerrainScenario:
    return TerrainScenario(
        terrain_id=terrain_id,
        display_name=f"SCM pilot patch ({terrain_id})",
        terrain_type="granular",
        surface_hint="scm_pilot",
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


def build_pilot_scenario(rover_key: str, slope_deg: float, soil_key: str) -> PilotScenario:
    """Build one (rover, slope, soil) pilot scenario. Requires pychrono.vehicle."""
    import pychrono as chrono

    rover_spec = load_rover_spec(rover_key)
    soil_material = load_soil_material(soil_key)

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(_tilted_gravity(chrono, slope_deg))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    terrain_scenario = _make_flat_terrain_scenario(f"scm_pilot_{soil_key}", soil_material.material_id)
    terrain = build_scm_terrain(system, terrain_scenario, soil_material)

    spawn_z = 0.05
    spawn_frame = chrono.ChFramed(chrono.ChVector3d(-PATCH_DIMENSIONS_XYZ_M[0] / 2.0 + 0.2, 0.0, spawn_z), chrono.QUNIT)
    rover = build_rover_from_spec(system, rover_spec, spawn_frame=spawn_frame)

    return PilotScenario(
        system=system,
        terrain=terrain,
        rover=rover,
        rover_spec=rover_spec,
        soil_material=soil_material,
        slope_deg=slope_deg,
    )
