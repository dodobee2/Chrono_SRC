from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.backends import HeuristicBackend, MockChronoBackend
from src.integration_schemas import ControlProfile, ObstacleSpec, RoverSpec, TerrainScenario
from src.registries import ControlProfileRegistry, RoverRegistry, TerrainRegistry


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
            mu_eff=0.6,
            crr=0.08,
        )
    except ValueError as exc:
        assert "mass_kg" in str(exc)
    else:
        raise AssertionError("negative mass should fail validation")


def test_registries_load_sample_handoff_files() -> None:
    rover_registry = RoverRegistry(PROJECT_ROOT / "rover_models")
    terrain_registry = TerrainRegistry(PROJECT_ROOT / "terrain_scenarios")
    control_registry = ControlProfileRegistry(PROJECT_ROOT / "control_profiles")

    assert "main_rover_baseline" in rover_registry.ids()
    assert {"T01_flat", "T02_slope", "T03_single_rock", "T04_rock_field"}.issubset(set(terrain_registry.ids()))
    assert "nominal_traverse" in control_registry.ids()

    rover = rover_registry.load("main_rover_baseline")
    terrain = terrain_registry.load("T03_single_rock")
    control = control_registry.load("nominal_traverse")

    assert rover.mass_kg > 0
    assert terrain.max_obstacle_height_m > 0
    assert control.duration_s > 0


def test_heuristic_backend_returns_common_simulation_result() -> None:
    rover = RoverRegistry(PROJECT_ROOT / "rover_models").load("main_rover_baseline")
    terrain = TerrainRegistry(PROJECT_ROOT / "terrain_scenarios").load("T04_rock_field")
    control = ControlProfileRegistry(PROJECT_ROOT / "control_profiles").load("nominal_traverse")

    result = HeuristicBackend().run(rover, terrain, control)

    assert result.backend_name == "heuristic"
    assert result.status == "ok"
    assert result.rover_id == rover.rover_id
    assert result.terrain_id == terrain.terrain_id
    assert 0.0 <= result.final_risk <= 1.0
    assert "physics_f_req_n" in result.metrics


def test_mock_chrono_does_not_claim_real_chrono_execution() -> None:
    rover = RoverRegistry(PROJECT_ROOT / "rover_models").load("main_rover_baseline")
    terrain = TerrainRegistry(PROJECT_ROOT / "terrain_scenarios").load("T01_flat")
    control = ControlProfileRegistry(PROJECT_ROOT / "control_profiles").load("slow_survey")

    result = MockChronoBackend().run(rover, terrain, control)

    assert result.backend_name == "mock_chrono"
    assert result.status == "mock"
    assert result.metrics["mock_no_physics_engine_invoked"] == 1.0
    assert any("intentionally does not create" in note for note in result.notes)


def test_terrain_scenario_to_scout_measurement_uses_max_obstacle_height() -> None:
    scenario = TerrainScenario(
        terrain_id="unit",
        display_name="Unit",
        terrain_type="rocky",
        surface_hint="obstacle_step",
        slope_long_deg=0.0,
        slope_lat_deg=0.0,
        roughness_m=0.01,
        gap_width_m=0.0,
        obstacles=[
            ObstacleSpec("low", "rock", 0.0, 0.0, 0.02),
            ObstacleSpec("high", "rock", 0.1, 0.0, 0.08),
        ],
        scout_response={
            "scout_slip": 0.1,
            "scout_sinkage_m": 0.01,
            "scout_wheel_torque_nm": 0.5,
            "scout_cot": 0.6,
            "vibration_rms_g": 0.2,
        },
    )

    measurement = scenario.to_scout_measurement()
    assert measurement.obstacle_height_m == 0.08

