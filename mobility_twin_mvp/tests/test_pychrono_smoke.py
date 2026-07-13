from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.backends import evaluated_grade_counts
from src.chrono.availability import get_pychrono_availability
from src.chrono.pychrono_backend import PyChronoSmokeBackend
from src.chrono.smoke_scenario import run_smoke_scenario, validate_trajectory_schema
from src.integration_schemas import SimulationResult
from src.registries import ControlProfileRegistry, RoverRegistry, TerrainRegistry


def load_contract_inputs():
    rover = RoverRegistry(PROJECT_ROOT / "rover_models", repo_root=PROJECT_ROOT).load("main_rover_baseline")
    terrain = TerrainRegistry(PROJECT_ROOT / "terrain_scenarios", repo_root=PROJECT_ROOT).load("T01_flat")
    control = ControlProfileRegistry(PROJECT_ROOT / "control_profiles", repo_root=PROJECT_ROOT).load("slow_survey")
    return rover, terrain, control


def test_pychrono_availability_is_import_safe() -> None:
    availability = get_pychrono_availability()
    assert availability.python_executable
    assert isinstance(availability.pychrono_available, bool)
    assert isinstance(availability.vehicle_module_available, bool)
    assert availability.diagnostic_message


def test_trajectory_schema_validation_accepts_required_columns() -> None:
    validate_trajectory_schema(
        [
            {
                "time_s": 0.0,
                "position_x_m": 0.0,
                "position_y_m": 0.0,
                "position_z_m": 1.0,
                "velocity_x_mps": 0.0,
                "velocity_y_mps": 0.0,
                "velocity_z_mps": 0.0,
            }
        ]
    )


def test_pychrono_smoke_backend_not_evaluated_without_requiring_pychrono(tmp_path: Path) -> None:
    rover, terrain, control = load_contract_inputs()
    result = PyChronoSmokeBackend(artifact_root=tmp_path).run(rover, terrain, control)

    assert result.backend_name == "pychrono_smoke"
    assert result.model_status == "chrono_smoke"
    assert result.evaluation_state == "NOT_EVALUATED"
    assert result.final_risk is None
    assert result.grade == "NOT_EVALUATED"


def test_pychrono_smoke_result_is_not_in_risk_counts(tmp_path: Path) -> None:
    rover, terrain, control = load_contract_inputs()
    result = PyChronoSmokeBackend(artifact_root=tmp_path).run(rover, terrain, control)

    counts = evaluated_grade_counts([result])
    assert sum(counts.values()) == 0


def test_pychrono_smoke_result_json_round_trip(tmp_path: Path) -> None:
    rover, terrain, control = load_contract_inputs()
    result = PyChronoSmokeBackend(artifact_root=tmp_path).run(rover, terrain, control)
    restored = SimulationResult.from_mapping(json.loads(json.dumps(result.to_dict())))

    assert restored.backend_name == "pychrono_smoke"
    assert restored.final_risk is None
    assert restored.evaluation_state == "NOT_EVALUATED"


@pytest.mark.pychrono
def test_real_pychrono_smoke_scenario_when_available() -> None:
    availability = get_pychrono_availability()
    if not availability.pychrono_available:
        pytest.skip(availability.diagnostic_message)

    result = run_smoke_scenario()
    assert result.status == "completed"
    assert result.trajectory
    validate_trajectory_schema(result.trajectory)
    final_z = result.trajectory[-1]["position_z_m"]
    assert final_z < 0.25
    assert result.metrics["contact_detected"] is True

