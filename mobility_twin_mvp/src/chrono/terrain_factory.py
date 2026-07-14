"""Builds PyChrono terrain bodies from Contract v2 TerrainScenario/TerrainMaterialSpec.

Scope for this first pass (see README.md "Next Steps"):
    - rigid, flat, obstacle-free patches (T0-style)
    - SCM deformable soil patches (loose granular, T3/T4-style)

Rocky/uneven/gated/sloped terrain (T1/T2/obstacle zones) is NOT implemented
here. handoff/map.py (Jongmin, 2026-07-14) already builds a full 5-zone
competition arena for those cases directly against a raw ChSystem; it is not
yet driven by TerrainScenario. Extend build_terrain_from_scenario per zone
type using handoff/map.py as the reference implementation rather than
silently approximating rocky/sloped terrain as flat.

SCM default parameters are the loose, non-cohesive sand test values measured
in handoff/map.py (SCM_BEKER_*, SCM_MOHR_*, SCM_JANOSI_SHEAR,
SCM_ELASTIC_STIFFNESS, SCM_DAMPING) and are used only when
TerrainMaterialSpec.scm_parameters does not override a given key.

Contact material type must match the caller's system: build_rigid_flat_terrain
uses ChContactMaterialNSC (not SMC) because every verified-working system in
this project (rover_factory's vendored builder, smoke_scenario.py) uses
ChSystemNSC. This was originally SMC and the mismatch was a real, silent bug
(2026-07-14): the floor's collision shape existed and occasionally still
registered a contact count, but the mismatched contact method meant no real
collision force was ever applied, so bodies fell straight through it. Found
via src/experiments/rigid_transfer_pilot's real trajectory output (z climbing
to -3.86m over under a second, contact_count=0 throughout) -- the earlier
unit test only checked `floor.GetCollisionModel() is not None`, which is true
regardless of this bug and did not catch it. If you ever need SMC here,
change the whole call chain (system + rover contact materials) together, not
just the floor.
"""

from __future__ import annotations

from typing import Any

from ..integration_schemas import TerrainMaterialSpec, TerrainScenario

DEFAULT_RIGID_FRICTION = 0.8
DEFAULT_RESTITUTION = 0.02

DEFAULT_SCM_PARAMETERS: dict[str, float] = {
    "bekker_kphi": 1.5e5,
    "bekker_kc": 0.0,
    "bekker_n": 1.10,
    "mohr_cohesion": 0.0,
    "mohr_friction_angle_deg": 28.0,
    "janosi_shear_m": 0.02,
    "elastic_stiffness": 1.5e7,
    "damping": 2.0e4,
    "grid_spacing_m": 0.025,
}


def _scm_param(material: TerrainMaterialSpec, key: str) -> float:
    value = material.scm_parameters.get(key, DEFAULT_SCM_PARAMETERS[key])
    return float(value)


def build_rigid_flat_terrain(
    system: Any,
    terrain: TerrainScenario,
    material: TerrainMaterialSpec | None = None,
) -> Any:
    """Build a single flat rigid floor box sized by terrain.dimensions_xyz_m.

    Raises NotImplementedError for slope or obstacles rather than silently
    returning a flat floor for a scenario that is supposed to have structure.
    """
    import pychrono as chrono

    if abs(terrain.slope_long_deg) > 1e-6 or abs(terrain.slope_lat_deg) > 1e-6:
        raise NotImplementedError(
            f"build_rigid_flat_terrain does not apply slope yet "
            f"(terrain_id={terrain.terrain_id!r}, slope_long_deg={terrain.slope_long_deg}, "
            f"slope_lat_deg={terrain.slope_lat_deg}); see handoff/map.py "
            "create_slope_terrain for a reference implementation."
        )
    if terrain.obstacles:
        raise NotImplementedError(
            f"build_rigid_flat_terrain does not place obstacles yet "
            f"(terrain_id={terrain.terrain_id!r} has {len(terrain.obstacles)}); "
            "see handoff/map.py create_rock_zone for a reference implementation."
        )

    length_x, width_y, thickness = terrain.dimensions_xyz_m
    friction = (
        material.friction_nominal
        if material and material.friction_nominal is not None
        else DEFAULT_RIGID_FRICTION
    )
    restitution = (
        material.restitution if material and material.restitution is not None else DEFAULT_RESTITUTION
    )

    floor_material = chrono.ChContactMaterialNSC()
    floor_material.SetFriction(friction)
    floor_material.SetRestitution(restitution)

    floor = chrono.ChBodyEasyBox(length_x, width_y, thickness, 2000.0, True, True, floor_material)
    floor.SetName(f"{terrain.terrain_id}_rigid_floor")
    floor.SetPos(chrono.ChVector3d(0.0, 0.0, -thickness / 2.0))
    floor.SetFixed(True)
    floor.EnableCollision(True)
    system.Add(floor)
    return floor


def build_scm_terrain(
    system: Any,
    terrain: TerrainScenario,
    material: TerrainMaterialSpec,
) -> Any:
    """Build an SCM deformable terrain patch sized by terrain.dimensions_xyz_m.

    Soil parameters come from material.scm_parameters, falling back to the
    loose-sand values in DEFAULT_SCM_PARAMETERS (sourced from handoff/map.py).
    """
    import pychrono as chrono
    import pychrono.vehicle as veh

    length_x, width_y, _thickness = terrain.dimensions_xyz_m
    grid_spacing = _scm_param(material, "grid_spacing_m")

    scm = veh.SCMTerrain(system)
    scm.SetReferenceFrame(chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))
    scm.Initialize(length_x, width_y, grid_spacing)
    scm.SetSoilParameters(
        _scm_param(material, "bekker_kphi"),
        _scm_param(material, "bekker_kc"),
        _scm_param(material, "bekker_n"),
        _scm_param(material, "mohr_cohesion"),
        _scm_param(material, "mohr_friction_angle_deg"),
        _scm_param(material, "janosi_shear_m"),
        _scm_param(material, "elastic_stiffness"),
        _scm_param(material, "damping"),
    )
    scm.SetPlotType(veh.SCMTerrain.PLOT_NONE, 0.0, 1.0)
    return scm


def build_terrain_from_scenario(
    system: Any,
    terrain: TerrainScenario,
    material: TerrainMaterialSpec | None = None,
) -> Any:
    """Dispatch to a rigid or SCM terrain builder based on terrain/material type.

    Anything other than rigid-flat or SCM-granular raises NotImplementedError
    -- no silent fallback to a flat floor for terrain that should have
    rocks/slope/gates.
    """
    model_type = material.model_type if material else "rigid"
    if model_type == "scm" or terrain.terrain_type == "granular":
        if material is None:
            raise ValueError(f"SCM terrain requires a TerrainMaterialSpec (terrain_id={terrain.terrain_id!r})")
        return build_scm_terrain(system, terrain, material)
    if terrain.terrain_type == "rigid":
        return build_rigid_flat_terrain(system, terrain, material)
    raise NotImplementedError(
        f"terrain_factory has no builder for terrain_type={terrain.terrain_type!r} "
        f"model_type={model_type!r} (terrain_id={terrain.terrain_id!r}); "
        "see handoff/map.py for rocky/uneven/obstacle reference implementations."
    )
