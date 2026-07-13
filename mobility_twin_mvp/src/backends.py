from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict

from .integration_schemas import (
    ContactPairSpec,
    ControlProfile,
    MobilityMetrics,
    RoverSpec,
    ScoutObservation,
    SimulationResult,
    TerrainScenario,
)
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
        observation: ScoutObservation | None = None,
        contact_pair: ContactPairSpec | None = None,
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
        observation: ScoutObservation | None = None,
        contact_pair: ContactPairSpec | None = None,
    ) -> SimulationResult:
        resolved_observation = observation or terrain.legacy_observation(rover.rover_id, control.profile_id)
        if resolved_observation is None:
            return not_evaluated_result(
                backend_name=self.name,
                rover=rover,
                terrain=terrain,
                control=control,
                status="not_evaluated",
                model_status="heuristic",
                notes=[
                    "HeuristicBackend requires ScoutObservation. TerrainScenario is geometry-only in Contract v2.",
                    "No legacy scout_response was available for migration.",
                ],
            )

        if resolved_observation.terrain_id != terrain.terrain_id:
            raise ValueError("ScoutObservation terrain_id does not match TerrainScenario")
        if resolved_observation.control_profile_id != control.profile_id:
            raise ValueError("ScoutObservation control_profile_id does not match ControlProfile")

        mu_eff = contact_pair.mu_eff if contact_pair and contact_pair.mu_eff is not None else rover.fallback_mu_eff
        crr = contact_pair.crr_eff if contact_pair and contact_pair.crr_eff is not None else rover.fallback_crr
        if mu_eff is None or crr is None:
            return not_evaluated_result(
                backend_name=self.name,
                rover=rover,
                terrain=terrain,
                control=control,
                status="not_evaluated",
                model_status="heuristic",
                notes=[
                    "HeuristicBackend requires effective mu_eff and crr from ContactPairSpec or RoverSpec fallback values.",
                    "No assumed values were invented.",
                ],
            )

        measurement = resolved_observation.to_scout_measurement(surface_hint=terrain.surface_hint)
        patch_result = analyze_patch(measurement, rover.to_main_config(mu_eff=mu_eff, crr=crr), self.scout_reference)
        metrics = {
            **{f"physics_{name}": float(value) for name, value in asdict(patch_result.physics).items()},
            "target_speed_mps": control.target_speed_mps,
            "command_duration_s": control.duration_s,
            "terrain_obstacle_count": float(len(terrain.obstacles)),
            "observation_mean_slip": resolved_observation.mean_slip,
            "observation_max_slip": resolved_observation.max_slip,
            "observation_mean_sinkage_m": resolved_observation.mean_sinkage_m,
            "observation_max_sinkage_m": resolved_observation.max_sinkage_m,
            "contact_mu_eff": mu_eff,
            "contact_crr_eff": crr,
        }
        prediction_confidence = min(
            resolved_observation.geometry_confidence,
            resolved_observation.response_confidence,
            contact_pair.confidence if contact_pair else 0.35,
        )
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
            metrics_typed=MobilityMetrics(
                completed=True,
                simulation_time_s=control.duration_s,
                travel_distance_m=resolved_observation.travel_distance_m,
                mean_body_speed_mps=resolved_observation.mean_body_speed_mps,
                mean_slip=resolved_observation.mean_slip,
                max_slip=resolved_observation.max_slip,
                mean_sinkage_m=resolved_observation.mean_sinkage_m,
                max_sinkage_m=resolved_observation.max_sinkage_m,
                peak_wheel_torque_nm=resolved_observation.mean_wheel_torque_nm,
                min_tipover_margin_deg=patch_result.physics.tipover_margin_deg,
                chassis_collision=False,
                wheel_stall=patch_result.physics.predicted_main_slip >= 0.8,
                rollover=patch_result.physics.tipover_margin_deg <= 0,
                timeout=False,
            ),
            prediction_confidence=prediction_confidence,
            model_status="heuristic",
            evaluation_state="EVALUATED",
            failure_reasons=patch_result.hard_failure_reasons,
            hard_failure_reasons=patch_result.hard_failure_reasons,
            notes=[
                "Existing MVP heuristic backend; no Chrono rover, wheel, contact, or terrain model was invoked.",
                "Slip/sinkage scaling remains a concept-validation heuristic.",
            ],
        )


class MockChronoBackend(MobilityBackend):
    """Chrono integration placeholder that preserves the final contract without claiming evaluation."""

    name = "mock_chrono"

    def run(
        self,
        rover: RoverSpec,
        terrain: TerrainScenario,
        control: ControlProfile,
        observation: ScoutObservation | None = None,
        contact_pair: ContactPairSpec | None = None,
    ) -> SimulationResult:
        heuristic = HeuristicBackend().run(rover, terrain, control, observation=observation, contact_pair=contact_pair)
        artifacts = {
            "heuristic_reference_risk": heuristic.final_risk,
            "heuristic_reference_grade": heuristic.grade,
            "heuristic_reference_components": heuristic.risk_components,
        }
        return SimulationResult.new(
            backend_name=self.name,
            rover_id=rover.rover_id,
            terrain_id=terrain.terrain_id,
            control_profile_id=control.profile_id,
            status="mock",
            duration_s=control.duration_s,
            metrics={
                "mock_chrono_time_step_s": 0.01,
                "mock_requested_speed_mps": control.target_speed_mps,
                "mock_no_physics_engine_invoked": 1.0,
            },
            risk_components={},
            final_risk=None,
            grade="NOT_EVALUATED",
            metrics_typed=MobilityMetrics(completed=None, simulation_time_s=None),
            prediction_confidence=0.0,
            model_status="mock",
            evaluation_state="NOT_EVALUATED",
            failure_reasons=[],
            hard_failure_reasons=[],
            notes=[
                "MOCK CHRONO -- Chrono physics engine was not executed.",
                "Reference artifacts come from the heuristic backend and are not Chrono results.",
                "No CAD, collision, tire, SCM, DEM, or Chrono wheel model was created.",
            ],
            artifacts=artifacts,
        )


def not_evaluated_result(
    backend_name: str,
    rover: RoverSpec,
    terrain: TerrainScenario,
    control: ControlProfile,
    status: str,
    model_status: str,
    notes: list[str],
) -> SimulationResult:
    return SimulationResult.new(
        backend_name=backend_name,
        rover_id=rover.rover_id,
        terrain_id=terrain.terrain_id,
        control_profile_id=control.profile_id,
        status=status,
        duration_s=0.0,
        metrics={},
        risk_components={},
        final_risk=None,
        grade="NOT_EVALUATED",
        metrics_typed=MobilityMetrics(completed=None),
        prediction_confidence=0.0,
        model_status=model_status,
        evaluation_state="NOT_EVALUATED",
        failure_reasons=[],
        hard_failure_reasons=[],
        notes=notes,
    )


def evaluated_grade_counts(results: list[SimulationResult]) -> dict[str, int]:
    counts = {"Safe": 0, "Caution": 0, "Risk": 0, "Unknown": 0}
    for result in results:
        if result.evaluation_state != "EVALUATED":
            continue
        if result.grade in counts:
            counts[result.grade] += 1
    return counts


def make_backend(backend_id: str) -> MobilityBackend:
    if backend_id == "heuristic":
        return HeuristicBackend()
    if backend_id == "mock_chrono":
        return MockChronoBackend()
    if backend_id == "pychrono_smoke":
        from .chrono.pychrono_backend import PyChronoSmokeBackend

        return PyChronoSmokeBackend()
    raise ValueError(f"unknown backend: {backend_id}")
