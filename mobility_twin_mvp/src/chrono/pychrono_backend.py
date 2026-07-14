from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

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
from .smoke_scenario import SmokeScenarioResult, write_trajectory_csv


class PyChronoSmokeBackend(MobilityBackend):
    """Runs a real headless PyChrono smoke scenario in an isolated worker."""

    name = "pychrono_smoke"

    def __init__(self, artifact_root: Path | None = None, worker_timeout_s: float = 45.0) -> None:
        self.artifact_root = artifact_root or Path("data") / "chrono_smoke"
        self.worker_timeout_s = worker_timeout_s

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
            self._ensure_empty_trajectory(trajectory_csv)
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
                artifacts=self._artifacts(trajectory_csv, result_json, runner_log),
            )
            result_json.write_text(json.dumps(result.to_dict(), indent=2), encoding="utf-8")
            return result

        config = build_default_smoke_config()
        scenario_json = artifact_dir / "scenario.json"
        worker_raw_result = artifact_dir / "worker_smoke_result.json"
        scenario_json.write_text(json.dumps(config.to_dict(), indent=2), encoding="utf-8")
        command = [
            sys.executable,
            "-m",
            "src.chrono.pychrono_runner",
            "--scenario",
            str(scenario_json),
            "--output",
            str(worker_raw_result),
        ]

        try:
            completed = subprocess.run(
                command,
                cwd=Path.cwd(),
                capture_output=True,
                text=True,
                timeout=self.worker_timeout_s,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            result = self._worker_timeout_result(
                rover=rover,
                terrain=terrain,
                control=control,
                trajectory_csv=trajectory_csv,
                result_json=result_json,
                runner_log=runner_log,
                command=command,
                timeout_s=self.worker_timeout_s,
                stdout=exc.stdout,
                stderr=exc.stderr,
            )
            result_json.write_text(json.dumps(result.to_dict(), indent=2), encoding="utf-8")
            return result

        runner_log.write_text(
            "command: " + " ".join(command) + "\n"
            + f"returncode: {completed.returncode}\n"
            + "--- stdout ---\n"
            + (completed.stdout or "")
            + "\n--- stderr ---\n"
            + (completed.stderr or ""),
            encoding="utf-8",
        )
        if not worker_raw_result.exists():
            result = self._worker_failed_result(
                rover=rover,
                terrain=terrain,
                control=control,
                trajectory_csv=trajectory_csv,
                result_json=result_json,
                runner_log=runner_log,
                message="PyChrono worker did not produce result JSON.",
            )
            result_json.write_text(json.dumps(result.to_dict(), indent=2), encoding="utf-8")
            return result

        smoke = SmokeScenarioResult(**json.loads(worker_raw_result.read_text(encoding="utf-8")))
        if smoke.trajectory:
            write_trajectory_csv(smoke.trajectory, trajectory_csv)
        else:
            self._ensure_empty_trajectory(trajectory_csv)

        status = smoke.status if smoke.status in {"completed", "timeout"} else "failed"
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
            artifacts=self._artifacts(trajectory_csv, result_json, runner_log),
        )
        result_json.write_text(json.dumps(result.to_dict(), indent=2), encoding="utf-8")
        return result

    def _worker_timeout_result(
        self,
        rover: RoverSpec,
        terrain: TerrainScenario,
        control: ControlProfile,
        trajectory_csv: Path,
        result_json: Path,
        runner_log: Path,
        command: list[str],
        timeout_s: float,
        stdout: str | bytes | None,
        stderr: str | bytes | None,
    ) -> SimulationResult:
        runner_log.write_text(
            "command: " + " ".join(command) + "\n"
            + f"timeout_s: {timeout_s}\n"
            + "--- stdout ---\n"
            + self._decode(stdout)
            + "\n--- stderr ---\n"
            + self._decode(stderr),
            encoding="utf-8",
        )
        self._ensure_empty_trajectory(trajectory_csv)
        message = f"PyChrono smoke worker timed out after {timeout_s:.1f} s."
        return SimulationResult.new(
            backend_name=self.name,
            rover_id=rover.rover_id,
            terrain_id=terrain.terrain_id,
            control_profile_id=control.profile_id,
            status="timeout",
            duration_s=0.0,
            metrics={"wall_time_s": timeout_s, "contact_detected": False, "timed_out": True},
            risk_components={},
            final_risk=None,
            grade="NOT_EVALUATED",
            metrics_typed=MobilityMetrics(completed=False, timeout=True),
            prediction_confidence=0.0,
            model_status="chrono_smoke",
            evaluation_state="NOT_EVALUATED",
            failure_reasons=[message],
            notes=["PyChrono smoke worker timed out before producing a result."],
            artifacts=self._artifacts(trajectory_csv, result_json, runner_log),
        )

    def _worker_failed_result(
        self,
        rover: RoverSpec,
        terrain: TerrainScenario,
        control: ControlProfile,
        trajectory_csv: Path,
        result_json: Path,
        runner_log: Path,
        message: str,
    ) -> SimulationResult:
        self._ensure_empty_trajectory(trajectory_csv)
        return SimulationResult.new(
            backend_name=self.name,
            rover_id=rover.rover_id,
            terrain_id=terrain.terrain_id,
            control_profile_id=control.profile_id,
            status="failed",
            duration_s=0.0,
            metrics={"wall_time_s": 0.0, "contact_detected": False},
            risk_components={},
            final_risk=None,
            grade="NOT_EVALUATED",
            metrics_typed=MobilityMetrics(completed=False),
            prediction_confidence=0.0,
            model_status="chrono_smoke",
            evaluation_state="NOT_EVALUATED",
            failure_reasons=[message],
            notes=["PyChrono smoke worker failed before producing a result."],
            artifacts=self._artifacts(trajectory_csv, result_json, runner_log),
        )

    @staticmethod
    def _decode(value: str | bytes | None) -> str:
        if value is None:
            return ""
        if isinstance(value, bytes):
            return value.decode(errors="replace")
        return value

    @staticmethod
    def _ensure_empty_trajectory(path: Path) -> None:
        if not path.exists():
            path.write_text(
                "time_s,position_x_m,position_y_m,position_z_m,velocity_x_mps,velocity_y_mps,velocity_z_mps\n",
                encoding="utf-8",
            )

    @staticmethod
    def _artifacts(trajectory_csv: Path, result_json: Path, runner_log: Path) -> dict[str, str]:
        return {
            "trajectory_csv": str(trajectory_csv),
            "result_json": str(result_json),
            "runner_log": str(runner_log),
        }
