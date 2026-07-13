from __future__ import annotations

from dataclasses import asdict, dataclass

import pandas as pd

from .mobility_physics import MobilityPhysicsResult, calculate_mobility_physics
from .schemas import MainRoverConfig, REQUIRED_MEASUREMENT_COLUMNS, ScoutMeasurement, ScoutReferenceConfig
from .terrain_classifier import TerrainClassification, classify_terrain


RISK_WEIGHTS = {
    "traction": 0.22,
    "tipover": 0.18,
    "obstacle": 0.15,
    "gap": 0.10,
    "slip": 0.15,
    "sinkage": 0.10,
    "energy": 0.05,
    "vibration": 0.05,
}


@dataclass(frozen=True)
class PatchRiskResult:
    measurement: ScoutMeasurement
    terrain: TerrainClassification
    physics: MobilityPhysicsResult
    risk_components: dict[str, float]
    final_risk: float
    grade: str
    hard_failure_reasons: list[str]


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def ramp(value: float, low: float, high: float) -> float:
    if high <= low:
        raise ValueError("high must be greater than low")
    return clamp01((value - low) / (high - low))


def calculate_risk_components(
    measurement: ScoutMeasurement,
    terrain: TerrainClassification,
    physics: MobilityPhysicsResult,
) -> dict[str, float]:
    """Return individual 0..1 risk terms."""
    traction_risk = 1.0 if physics.traction_margin_n <= 0 else clamp01(1.0 - physics.traction_margin_ratio / 2.0)
    tipover_risk = ramp(abs(measurement.slope_lat_deg), physics.beta_crit_deg * 0.35, physics.beta_crit_deg)
    obstacle_risk = max(ramp(physics.obstacle_ratio, 0.25, 1.0), ramp(physics.clearance_ratio, 0.35, 1.0))
    gap_risk = ramp(physics.gap_ratio, 0.25, 1.0)
    slip_risk = ramp(physics.predicted_main_slip, 0.08, 0.8)
    sinkage_risk = ramp(physics.predicted_main_sinkage_m, 0.015, 0.10)
    energy_risk = ramp(measurement.scout_cot, 0.45, 2.5)
    vibration_risk = ramp(measurement.vibration_rms_g, 0.15, 1.2)
    uncertainty_risk = clamp01(1.0 - terrain.confidence)

    return {
        "traction": traction_risk,
        "tipover": tipover_risk,
        "obstacle": obstacle_risk,
        "gap": gap_risk,
        "slip": slip_risk,
        "sinkage": sinkage_risk,
        "energy": energy_risk,
        "vibration": vibration_risk,
        "uncertainty": uncertainty_risk,
    }


def hard_failure_reasons(
    measurement: ScoutMeasurement,
    config: MainRoverConfig,
    physics: MobilityPhysicsResult,
) -> list[str]:
    reasons: list[str] = []
    if physics.f_avail_n <= physics.f_req_n:
        reasons.append("F_avail <= F_req")
    if physics.tipover_margin_deg <= 0:
        reasons.append("tipover_margin_deg <= 0")
    if measurement.obstacle_height_m >= config.ground_clearance_m:
        reasons.append("obstacle_height >= ground_clearance")
    if measurement.gap_width_m >= 2.0 * config.wheel_radius_m:
        reasons.append("gap_width >= wheel_diameter")
    if physics.predicted_main_slip >= 0.8:
        reasons.append("predicted_main_slip >= 0.8")
    return reasons


def fuse_final_risk(risk_components: dict[str, float], weights: dict[str, float] | None = None) -> float:
    weights = weights or RISK_WEIGHTS
    weighted_sum = sum(risk_components[name] * weight for name, weight in weights.items())
    return clamp01(weighted_sum / sum(weights.values()))


def grade_risk(final_risk: float, hard_failures: list[str], unknown: bool = False) -> str:
    if unknown:
        return "Unknown"
    if hard_failures or final_risk >= 0.65:
        return "Risk"
    if final_risk >= 0.35:
        return "Caution"
    return "Safe"


def analyze_patch(
    measurement: ScoutMeasurement,
    config: MainRoverConfig,
    scout_reference: ScoutReferenceConfig,
) -> PatchRiskResult:
    terrain = classify_terrain(measurement)
    physics = calculate_mobility_physics(measurement, config, scout_reference)
    components = calculate_risk_components(measurement, terrain, physics)
    final_risk = fuse_final_risk(components)
    failures = hard_failure_reasons(measurement, config, physics)
    unknown = terrain.label == "unknown"
    return PatchRiskResult(
        measurement=measurement,
        terrain=terrain,
        physics=physics,
        risk_components=components,
        final_risk=final_risk,
        grade=grade_risk(final_risk, failures, unknown),
        hard_failure_reasons=failures,
    )


def analyze_dataframe(
    frame: pd.DataFrame,
    config: MainRoverConfig,
    scout_reference: ScoutReferenceConfig,
) -> pd.DataFrame:
    missing_columns = [column for column in REQUIRED_MEASUREMENT_COLUMNS if column not in frame.columns]
    if missing_columns:
        raise ValueError(f"missing required columns: {', '.join(missing_columns)}")

    rows: list[dict[str, object]] = []
    for record in frame[REQUIRED_MEASUREMENT_COLUMNS].to_dict(orient="records"):
        result = analyze_patch(ScoutMeasurement.from_mapping(record), config, scout_reference)
        row = {
            **asdict(result.measurement),
            "terrain_label": result.terrain.label,
            "terrain_reason": result.terrain.reason,
            "terrain_confidence": result.terrain.confidence,
            **{f"risk_{name}": value for name, value in result.risk_components.items()},
            "final_risk": result.final_risk,
            "risk_score": int(round(result.final_risk * 100)),
            "grade": result.grade,
            "hard_failure_reasons": "; ".join(result.hard_failure_reasons),
            **{f"physics_{name}": value for name, value in asdict(result.physics).items()},
        }
        rows.append(row)
    return pd.DataFrame(rows)

