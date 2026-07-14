from __future__ import annotations

import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.chrono.rover_factory import to_chrono_rover_spec, to_sim_config
from src.chrono.terrain_factory import DEFAULT_SCM_PARAMETERS
from src.integration_schemas import TerrainGeometrySpec, TerrainScenario
from src.registries import RoverRegistry, TerrainMaterialRegistry, TerrainRegistry


def make_flat_terrain(terrain_id: str = "test_flat") -> TerrainScenario:
    return TerrainScenario(
        terrain_id=terrain_id,
        display_name="Test Flat Patch",
        terrain_type="rigid",
        surface_hint="rigid_flat",
        geometry=TerrainGeometrySpec.from_mapping({"frame_id": terrain_id}),
        material_id="regolith_assumed_v0",
        dimensions_xyz_m=(2.0, 2.0, 0.2),
        frame_id=terrain_id,
        random_seed=1,
    )


def rover_registry() -> RoverRegistry:
    return RoverRegistry(PROJECT_ROOT / "rover_models", repo_root=PROJECT_ROOT)


def terrain_material_registry() -> TerrainMaterialRegistry:
    return TerrainMaterialRegistry(PROJECT_ROOT / "terrain_materials", repo_root=PROJECT_ROOT)


def test_handoff_rover_specs_load_and_reference_real_handoff_files() -> None:
    registry = rover_registry()
    for rover_id in ("scout_v01", "main_v01"):
        rover = registry.load(rover_id)
        assert rover.rover_id == rover_id
        assert rover.mass_kg > 0
        assert rover.metadata["command_type"] in {"wheel_speed", "wheel_torque"}
        assert float(rover.metadata["max_wheel_speed_radps"]) > 0
        rover_yaml_dir = PROJECT_ROOT / "rover_models" / rover_id
        source_path = (rover_yaml_dir / rover.model_uri).resolve()
        assert source_path.exists(), f"model_uri does not point at a real file: {source_path}"


def test_rover_factory_translates_cg_height_and_metadata_fields() -> None:
    rover = rover_registry().load("main_v01")
    chrono_spec = to_chrono_rover_spec(rover)

    assert chrono_spec.cg_xyz_m == (0.0, 0.0, rover.cg_height_m)
    assert chrono_spec.mass_kg == rover.mass_kg
    assert chrono_spec.wheel_radius_m == rover.wheel_radius_m
    assert chrono_spec.command_type == "wheel_speed"
    assert chrono_spec.max_wheel_speed_radps == pytest.approx(15.0)
    assert chrono_spec.wheel_count == 4


def test_rover_factory_requires_max_wheel_speed_metadata() -> None:
    from dataclasses import replace

    rover = rover_registry().load("main_v01")
    rover_without_speed_limit = replace(
        rover, metadata={k: v for k, v in rover.metadata.items() if k != "max_wheel_speed_radps"}
    )
    with pytest.raises(ValueError, match="max_wheel_speed_radps"):
        to_chrono_rover_spec(rover_without_speed_limit)


def test_sim_config_rejects_unknown_metadata_fields() -> None:
    rover = rover_registry().load("main_v01")
    with pytest.raises(ValueError, match="Unknown SimConfig fields"):
        to_sim_config(rover, overrides={"not_a_real_field": 1})


def test_loose_sand_scm_material_loads_and_covers_default_keys() -> None:
    material = terrain_material_registry().load("loose_sand_scm_v0")
    assert material.model_type == "scm"
    for key in DEFAULT_SCM_PARAMETERS:
        assert key in material.scm_parameters


@pytest.mark.pychrono
def test_rigid_flat_terrain_rejects_sloped_scenario() -> None:
    from src.chrono.availability import get_pychrono_availability
    from src.chrono.terrain_factory import build_rigid_flat_terrain

    if not get_pychrono_availability().pychrono_available:
        pytest.skip("pychrono not available")

    import pychrono as chrono

    terrain = TerrainRegistry(PROJECT_ROOT / "terrain_scenarios", repo_root=PROJECT_ROOT).load("T01_flat")
    system = chrono.ChSystemNSC()
    with pytest.raises(NotImplementedError, match="slope"):
        build_rigid_flat_terrain(system, terrain)


@pytest.mark.pychrono
def test_rover_factory_builds_scout_and_main_with_matching_total_mass() -> None:
    from src.chrono.availability import get_pychrono_availability
    from src.chrono.rover_factory import build_rover_from_spec

    if not get_pychrono_availability().pychrono_available:
        pytest.skip("pychrono not available")

    import pychrono as chrono

    for rover_id in ("scout_v01", "main_v01"):
        rover = rover_registry().load(rover_id)
        system = chrono.ChSystemNSC()
        system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        instance = build_rover_from_spec(system, rover)
        assert instance.total_mass() == pytest.approx(rover.mass_kg, abs=1e-6)
        assert len(instance.wheels) == 4
        assert len(instance.motors) == rover.driven_wheel_count


@pytest.mark.pychrono
def test_rigid_flat_terrain_builds_a_collidable_floor() -> None:
    from src.chrono.availability import get_pychrono_availability
    from src.chrono.terrain_factory import build_rigid_flat_terrain

    if not get_pychrono_availability().pychrono_available:
        pytest.skip("pychrono not available")

    import pychrono as chrono

    flat = make_flat_terrain()
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    floor = build_rigid_flat_terrain(system, flat)
    assert floor.GetCollisionModel() is not None


@pytest.mark.pychrono
def test_rigid_flat_terrain_actually_stops_a_falling_body() -> None:
    """GetCollisionModel() is not None alone does not prove the floor collides --
    a floor with a mismatched contact material (e.g. SMC material under a
    ChSystemNSC) still has a non-null collision model but applies no real
    force, so a body falls straight through it. This regression was found
    2026-07-14 via src/experiments/rigid_transfer_pilot's real output
    (z_m falling to -3.86m, contact_count=0 throughout) after the weaker
    assertion above passed. Step real dynamics and check the box actually
    comes to rest on the floor instead of free-falling through it.
    """
    from src.chrono.availability import get_pychrono_availability
    from src.chrono.terrain_factory import build_rigid_flat_terrain

    if not get_pychrono_availability().pychrono_available:
        pytest.skip("pychrono not available")

    import pychrono as chrono

    flat = make_flat_terrain()
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    build_rigid_flat_terrain(system, flat)

    material = chrono.ChContactMaterialNSC()
    box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 500.0, True, True, material)
    box.SetPos(chrono.ChVector3d(0.0, 0.0, 0.5))
    box.SetFixed(False)
    box.EnableCollision(True)
    system.Add(box)

    for _ in range(2000):
        system.DoStepDynamics(0.001)

    assert box.GetPos().z > -0.1, (
        f"box fell through the floor to z={box.GetPos().z:.3f} -- terrain/rover contact material mismatch regression"
    )
    assert system.GetNumContacts() >= 1


@pytest.mark.pychrono
def test_scm_terrain_builds_a_deformable_patch() -> None:
    from src.chrono.availability import get_pychrono_availability

    if not get_pychrono_availability().pychrono_available:
        pytest.skip("pychrono not available")
    try:
        import pychrono.vehicle  # noqa: F401
    except ImportError as exc:
        pytest.skip(
            f"pychrono.vehicle failed to import in this environment ({exc}); "
            "SCMTerrain lives in pychrono.vehicle. This is an environment/install "
            "issue, not a terrain_factory bug -- see docs/ENVIRONMENT_SETUP.md."
        )

    from src.chrono.terrain_factory import build_scm_terrain

    import pychrono as chrono

    flat = make_flat_terrain()
    material = terrain_material_registry().load("loose_sand_scm_v0")
    scm_system = chrono.ChSystemSMC()
    scm_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    scm = build_scm_terrain(scm_system, flat, material)
    assert scm is not None
