from __future__ import annotations

import json
import sys
import warnings
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.backends import HeuristicBackend, MockChronoBackend, evaluated_grade_counts
from src.integration_schemas import (
    ContactPairSpec,
    ObstacleSpec,
    Pose3D,
    RoverSpec,
    SimulationResult,
    TerrainGeometrySpec,
    TerrainScenario,
)
from src.registries import (
    ContactPairRegistry,
    ControlProfileRegistry,
    ObservationRegistry,
    RoverRegistry,
    TerrainMaterialRegistry,
    TerrainRegistry,
)


def registries() -> tuple[
    RoverRegistry,
    TerrainRegistry,
    ControlProfileRegistry,
    ObservationRegistry,
    TerrainMaterialRegistry,
    ContactPairRegistry,
]:
    return (
        RoverRegistry(PROJECT_ROOT / "rover_models", repo_root=PROJECT_ROOT),
        TerrainRegistry(PROJECT_ROOT / "terrain_scenarios", repo_root=PROJECT_ROOT),
        ControlProfileRegistry(PROJECT_ROOT / "control_profiles", repo_root=PROJECT_ROOT),
        ObservationRegistry(PROJECT_ROOT / "observations", repo_root=PROJECT_ROOT),
        TerrainMaterialRegistry(PROJECT_ROOT / "terrain_materials", repo_root=PROJECT_ROOT),
        ContactPairRegistry(PROJECT_ROOT / "contact_pairs", repo_root=PROJECT_ROOT),
    )


def test_schema_validation_rejects_negative_mass() -> None:
    try:
        RoverSpec(
            rover_id="bad",
            display_name="Bad",
            mass_kg=-1.0,
            wheel_radius_m=0.1,
            wheel_width_m=0.05,
            wheelbase_m=0.5,
            track_width_m=0.4,
            cg_height_m=0.2,
            ground_clearance_m=0.1,
            driven_wheel_count=4,
            max_wheel_torque_nm=10.0,
            wheel_material_id="wheel",
            wheel_contact_model="placeholder",
            fallback_mu_eff=0.6,
            fallback_crr=0.08,
        )
    except ValueError as exc:
        assert "mass_kg" in str(exc)
    else:
        raise AssertionError("negative mass should fail validation")


def test_terrain_scenario_and_scout_observation_load_independently() -> None:
    _, terrain_registry, _, observation_registry, _, _ = registries()
    terrain = terrain_registry.load("T04_rock_field")
    observation = observation_registry.load("O04_rock_field_nominal")

    assert terrain.terrain_id == observation.terrain_id
    assert terrain.legacy_scout_response
    assert observation.mean_slip > 0
    assert observation.observation_state == "SCOUT_TRAVERSED"


def test_legacy_scout_response_yaml_is_migrated_with_warning() -> None:
    _, terrain_registry, control_registry, _, _, _ = registries()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        terrain = terrain_registry.load("T01_flat")
        observation = terrain.legacy_observation("scout_placeholder_v0", "slow_survey")
    assert observation is not None
    assert observation.terrain_id == "T01_flat"
    assert observation.mean_slip == terrain.legacy_scout_response["scout_slip"]
    assert control_registry.load("slow_survey").profile_id == observation.control_profile_id
    assert any("deprecated" in str(item.message) or "legacy" in str(item.message) for item in caught)


def test_legacy_mu_eff_crr_are_migrated_to_fallback_values() -> None:
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        rover = RoverSpec.from_mapping(
            {
                "rover_id": "legacy_rover",
                "display_name": "Legacy Rover",
                "mass_kg": 10,
                "wheel_radius_m": 0.1,
                "wheel_width_m": 0.04,
                "wheelbase_m": 0.4,
                "track_width_m": 0.3,
                "cg_height_m": 0.2,
                "ground_clearance_m": 0.08,
                "driven_wheel_count": 4,
                "max_wheel_torque_nm": 5,
                "mu_eff": 0.55,
                "crr": 0.07,
            }
        )
    assert rover.fallback_mu_eff == 0.55
    assert rover.fallback_crr == 0.07
    assert any("deprecated" in str(item.message) for item in caught)


def test_terrain_geometry_pose_and_asset_uri_are_parsed() -> None:
    geometry = TerrainGeometrySpec.from_mapping(
        {
            "source_type": "mesh",
            "asset_uri": "assets/terrain.obj",
            "factory_uri": "factories.terrain:build",
            "scale_xyz": [1, 2, 3],
            "frame_id": "terrain",
            "coordinate_convention": "Chrono_XForward_YLeft_ZUp",
            "origin_pose": {"xyz_m": [1, 2, 3], "rpy_deg": [0, 0, 10]},
        }
    )
    assert geometry.asset_uri == "assets/terrain.obj"
    assert geometry.origin_pose.xyz_m == (1.0, 2.0, 3.0)
    assert geometry.scale_xyz == (1.0, 2.0, 3.0)


def test_contact_pair_references_wheel_and_terrain_material() -> None:
    rover_registry, terrain_registry, _, _, material_registry, contact_registry = registries()
    rover = rover_registry.load("main_rover_baseline")
    terrain = terrain_registry.load("T04_rock_field")
    pair = contact_registry.load("wheel_rubber_rock_assumed_v0")

    contact_registry.validate_references(pair, rover, terrain, material_registry)
    assert pair.wheel_material_id == rover.wheel_material_id
    assert pair.terrain_material_id == terrain.material_id


def test_confidence_out_of_range_raises_validation_error() -> None:
    try:
        ContactPairSpec(
            contact_pair_id="bad",
            wheel_material_id="wheel",
            terrain_material_id="terrain",
            mu_eff=0.5,
            crr_eff=0.05,
            source="assumed",
            confidence=1.5,
        )
    except ValueError as exc:
        assert "confidence" in str(exc)
    else:
        raise AssertionError("confidence > 1 should fail validation")


def test_mock_chrono_backend_returns_not_evaluated() -> None:
    rover_registry, terrain_registry, control_registry, observation_registry, _, contact_registry = registries()
    rover = rover_registry.load("main_rover_baseline")
    terrain = terrain_registry.load("T01_flat")
    control = control_registry.load("slow_survey")
    observation = observation_registry.load("O01_flat_slow")
    contact = contact_registry.load("wheel_rubber_regolith_assumed_v0")

    result = MockChronoBackend().run(rover, terrain, control, observation=observation, contact_pair=contact)

    assert result.backend_name == "mock_chrono"
    assert result.status == "mock"
    assert result.model_status == "mock"
    assert result.evaluation_state == "NOT_EVALUATED"
    assert result.final_risk is None
    assert result.grade == "NOT_EVALUATED"
    assert result.prediction_confidence == 0.0
    assert result.hard_failure_reasons == []
    assert "heuristic_reference_risk" in result.artifacts


def test_mock_chrono_result_is_not_counted_as_safe_or_risk() -> None:
    rover_registry, terrain_registry, control_registry, observation_registry, _, contact_registry = registries()
    rover = rover_registry.load("main_rover_baseline")
    terrain = terrain_registry.load("T01_flat")
    control = control_registry.load("slow_survey")
    observation = observation_registry.load("O01_flat_slow")
    contact = contact_registry.load("wheel_rubber_regolith_assumed_v0")

    evaluated = HeuristicBackend().run(rover, terrain, control, observation=observation, contact_pair=contact)
    mock = MockChronoBackend().run(rover, terrain, control, observation=observation, contact_pair=contact)

    counts = evaluated_grade_counts([evaluated, mock])
    assert sum(counts.values()) == 1
    assert counts[evaluated.grade] == 1


def test_heuristic_backend_uses_observation_input() -> None:
    rover_registry, terrain_registry, control_registry, observation_registry, _, contact_registry = registries()
    rover = rover_registry.load("main_rover_baseline")
    terrain = terrain_registry.load("T04_rock_field")
    control = control_registry.load("nominal_traverse")
    observation = observation_registry.load("O04_rock_field_nominal")
    contact = contact_registry.load("wheel_rubber_rock_assumed_v0")

    result = HeuristicBackend().run(rover, terrain, control, observation=observation, contact_pair=contact)

    assert result.evaluation_state == "EVALUATED"
    assert result.metrics["observation_mean_slip"] == observation.mean_slip
    assert result.metrics_typed.mean_slip == observation.mean_slip
    assert 0.0 <= result.final_risk <= 1.0


def test_all_sample_registries_load() -> None:
    rover_registry, terrain_registry, control_registry, observation_registry, material_registry, contact_registry = registries()

    assert rover_registry.load_all()
    assert terrain_registry.load_all()
    assert control_registry.load_all()
    assert observation_registry.load_all()
    assert material_registry.load_all()
    assert contact_registry.load_all()


def test_simulation_result_json_round_trip() -> None:
    rover_registry, terrain_registry, control_registry, observation_registry, _, contact_registry = registries()
    rover = rover_registry.load("main_rover_baseline")
    terrain = terrain_registry.load("T01_flat")
    control = control_registry.load("slow_survey")
    observation = observation_registry.load("O01_flat_slow")
    contact = contact_registry.load("wheel_rubber_regolith_assumed_v0")

    result = HeuristicBackend().run(rover, terrain, control, observation=observation, contact_pair=contact)
    payload = json.loads(json.dumps(result.to_dict()))
    restored = SimulationResult.from_mapping(payload)

    assert restored.experiment_id == result.experiment_id
    assert restored.metrics_typed.mean_slip == result.metrics_typed.mean_slip
    assert restored.final_risk == result.final_risk


def test_terrain_scenario_max_obstacle_height_uses_v2_dimensions() -> None:
    scenario = TerrainScenario(
        terrain_id="unit",
        display_name="Unit",
        terrain_type="rocky",
        surface_hint="obstacle_step",
        geometry=TerrainGeometrySpec(
            source_type="procedural",
            asset_uri=None,
            factory_uri=None,
            scale_xyz=(1.0, 1.0, 1.0),
            frame_id="terrain",
            coordinate_convention="Chrono_XForward_YLeft_ZUp",
            origin_pose=Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ),
        material_id="rock_assumed_v0",
        dimensions_xyz_m=(1.0, 1.0, 0.2),
        frame_id="terrain",
        random_seed=1,
        obstacles=[
            ObstacleSpec("low", "rock", Pose3D((0, 0, 0), (0, 0, 0)), (0.1, 0.1, 0.02)),
            ObstacleSpec("high", "rock", Pose3D((0.1, 0, 0), (0, 0, 0)), (0.1, 0.1, 0.08)),
        ],
    )

    assert scenario.max_obstacle_height_m == 0.08

