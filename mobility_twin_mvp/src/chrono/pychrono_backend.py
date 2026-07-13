from __future__ import annotations

import json
from pathlib import Path

from ..backends import MobilityBackend
from ..integration_schemas import (
    ContactPairSpec,
    ControlProfile,
    MobilityMetrics,
    RoverSpec,
    ScoutObservation,
    SimulationResult,
    TerrainScenario,
)
from .availability import get_pychrono_availability
from .result_extractor import mobility_metrics_from_smoke
from .scenario_builder import build_default_smoke_config
from .smoke_scenario import run_smoke_scenario, write_trajectory_csv


class PyChronoSmokeBackend(MobilityBackend):
    """Runs a real headless PyChrono smoke scenario when PyChrono is available."""

    name = "pychrono_smoke"

    def __init__(self, artifact_root: Path | None = None) -> None:
        self.artifact_root = artifact_root or Path("data") / "chrono_smoke"

    def run(
        self,
        rover: RoverSpec,
        terrain: TerrainScenario,
        control: ControlProfile,
        observation: ScoutObservation | None = None,
        contact_pair: ContactPairSpec | None = None,
    ) -> SimulationResult:
        artifact_dir = self.artifact_root
        artifact_dir.mkdir(parents=True, exist_ok=True)
        trajectory_csv = artifact_dir / "trajectory.csv"
        result_json = artifact_dir / "result.json"
        runner_log = artifact_dir / "runner.log"
        availability = get_pychrono_availability()
        if not availability.pychrono_available:
            runner_log.write_text(availability.diagnostic_message, encoding="utf-8")
            if not trajectory_csv.exists():
                trajectory_csv.write_text(
                    "time_s,position_x_m,position_y_m,position_z_m,velocity_x_mps,velocity_y_mps,velocity_z_mps\n",
                    encoding="utf-8",
                )
            result = SimulationResult.new(
                backend_name=self.name,
                rover_id=rover.rover_id,
                terrain_id=terrain.terrain_id,
                control_profile_id=control.profile_id,
                status="failed",
                duration_s=0.0,
                metrics={},
                risk_components={},
                final_risk=None,
                grade="NOT_EVALUATED",
                metrics_typed=MobilityMetrics(completed=False),
                prediction_confidence=0.0,
                model_status="chrono_smoke",
                evaluation_state="NOT_EVALUATED",
                failure_reasons=[availability.diagnostic_message],
                notes=["PyChrono smoke could not run because PyChrono is unavailable."],
                artifacts={
                    "trajectory_csv": str(trajectory_csv),
                    "result_json": str(result_json),
                    "runner_log": str(runner_log),
                },
            )
            result_json.write_text(json.dumps(result.to_dict(), indent=2), encoding="utf-8")
            return result

        config = build_default_smoke_config()
        smoke = run_smoke_scenario(config)
        if smoke.trajectory:
            write_trajectory_csv(smoke.trajectory, trajectory_csv)
        elif not trajectory_csv.exists():
            trajectory_csv.write_text(
                "time_s,position_x_m,position_y_m,position_z_m,velocity_x_mps,velocity_y_mps,velocity_z_mps\n",
                encoding="utf-8",
            )
        runner_log.write_text(smoke.runner_log, encoding="utf-8")

        status = "completed" if smoke.status == "completed" else "failed"
        result = SimulationResult.new(
            backend_name=self.name,
            rover_id=rover.rover_id,
            terrain_id=terrain.terrain_id,
            control_profile_id=control.profile_id,
            status=status,
            duration_s=float(smoke.metrics.get("simulation_time_s", 0.0) or 0.0),
            metrics=smoke.metrics,
            risk_components={},
            final_risk=None,
            grade="NOT_EVALUATED",
            metrics_typed=mobility_metrics_from_smoke(smoke),
            prediction_confidence=0.0,
            model_status="chrono_smoke",
            evaluation_state="NOT_EVALUATED",
            failure_reasons=[] if smoke.status == "completed" else [smoke.error or "PyChrono smoke failed"],
            notes=[
                "PyChrono smoke uses a 1 kg falling box and fixed rigid floor.",
                "It is an environment/backend smoke check, not a rover mobility risk evaluation.",
            ],
            artifacts={
                "trajectory_csv": str(trajectory_csv),
                "result_json": str(result_json),
                "runner_log": str(runner_log),
            },
        )
        result_json.write_text(json.dumps(result.to_dict(), indent=2), encoding="utf-8")
        return result
