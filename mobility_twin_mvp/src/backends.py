from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict

from .integration_schemas import ControlProfile, RoverSpec, SimulationResult, TerrainScenario
from .risk_fusion import analyze_patch
from .schemas import ScoutReferenceConfig


class MobilityBackend(ABC):
    """Common simulation backend contract for heuristic and future Chrono adapters."""

    name: str

    @abstractmethod
    def run(
        self,
        rover: RoverSpec,
        terrain: TerrainScenario,
        control: ControlProfile,
    ) -> SimulationResult:
        raise NotImplementedError


class HeuristicBackend(MobilityBackend):
    """Wrap the existing MVP equations behind the integration backend interface."""

    name = "heuristic"

    def __init__(self, scout_reference: ScoutReferenceConfig | None = None) -> None:
        self.scout_reference = scout_reference or ScoutReferenceConfig()

    def run(
        self,
        rover: RoverSpec,
        terrain: TerrainScenario,
        control: ControlProfile,
    ) -> SimulationResult:
        measurement = terrain.to_scout_measurement()
        patch_result = analyze_patch(measurement, rover.to_main_config(), self.scout_reference)
        metrics = {
            **{f"physics_{name}": float(value) for name, value in asdict(patch_result.physics).items()},
            "target_speed_mps": control.target_speed_mps,
            "command_duration_s": control.duration_s,
            "terrain_obstacle_count": float(len(terrain.obstacles)),
        }
        return SimulationResult.new(
            backend_name=self.name,
            rover_id=rover.rover_id,
            terrain_id=terrain.terrain_id,
            control_profile_id=control.profile_id,
            status="ok",
            duration_s=control.duration_s,
            metrics=metrics,
            risk_components=patch_result.risk_components,
            final_risk=patch_result.final_risk,
            grade=patch_result.grade,
            hard_failure_reasons=patch_result.hard_failure_reasons,
            notes=[
                "Existing MVP heuristic backend; no Chrono rover, wheel, contact, or terrain model was invoked.",
                "Slip/sinkage scaling remains a concept-validation heuristic.",
            ],
        )


class MockChronoBackend(MobilityBackend):
    """Chrono integration placeholder that preserves the final contract."""

    name = "mock_chrono"

    def run(
        self,
        rover: RoverSpec,
        terrain: TerrainScenario,
        control: ControlProfile,
    ) -> SimulationResult:
        heuristic = HeuristicBackend().run(rover, terrain, control)
        metrics = dict(heuristic.metrics)
        metrics.update(
            {
                "mock_chrono_time_step_s": 0.01,
                "mock_requested_speed_mps": control.target_speed_mps,
                "mock_no_physics_engine_invoked": 1.0,
            }
        )
        notes = list(heuristic.notes) + [
            "MockChronoBackend intentionally does not create CAD, collision, tire, SCM, DEM, or Chrono wheel models.",
            "Replace this class with a PyChrono adapter after rover and terrain factories are delivered.",
        ]
        return SimulationResult.new(
            backend_name=self.name,
            rover_id=rover.rover_id,
            terrain_id=terrain.terrain_id,
            control_profile_id=control.profile_id,
            status="mock",
            duration_s=control.duration_s,
            metrics=metrics,
            risk_components=heuristic.risk_components,
            final_risk=heuristic.final_risk,
            grade=heuristic.grade,
            hard_failure_reasons=heuristic.hard_failure_reasons,
            notes=notes,
        )


def make_backend(backend_id: str) -> MobilityBackend:
    if backend_id == "heuristic":
        return HeuristicBackend()
    if backend_id == "mock_chrono":
        return MockChronoBackend()
    raise ValueError(f"unknown backend: {backend_id}")

