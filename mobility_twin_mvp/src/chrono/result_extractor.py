from __future__ import annotations

from .smoke_scenario import SmokeScenarioResult
from ..integration_schemas import MobilityMetrics


def mobility_metrics_from_smoke(result: SmokeScenarioResult) -> MobilityMetrics:
    metrics = result.metrics
    return MobilityMetrics(
        completed=result.status == "completed",
        simulation_time_s=_float_or_none(metrics.get("simulation_time_s")),
        travel_distance_m=_float_or_none(metrics.get("travel_distance_m")),
        mean_body_speed_mps=_float_or_none(metrics.get("mean_body_speed_mps")),
    )


def _float_or_none(value: object) -> float | None:
    if value is None:
        return None
    return float(value)

