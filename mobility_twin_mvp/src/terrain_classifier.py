from __future__ import annotations

from dataclasses import dataclass

from .schemas import ScoutMeasurement


SUPPORTED_TERRAIN_LABELS = {
    "rigid_flat",
    "rocky_rough",
    "granular",
    "rigid_slope",
    "granular_slope",
    "obstacle_step",
    "mixed",
    "unknown",
}


@dataclass(frozen=True)
class TerrainClassification:
    label: str
    reason: str
    confidence: float


def classify_terrain(measurement: ScoutMeasurement) -> TerrainClassification:
    """Classify terrain from scout measurements using transparent MVP rules."""
    hint = (measurement.surface_hint or "").strip().lower()
    if hint in SUPPORTED_TERRAIN_LABELS and hint != "unknown":
        return TerrainClassification(
            label=hint,
            reason=f"surface_hint supplied as '{hint}' and accepted as the primary label",
            confidence=0.9,
        )

    missing = _has_missing_numeric(measurement)
    if missing:
        return TerrainClassification("unknown", "one or more required numeric measurements are missing", 0.0)

    high_sinkage = measurement.scout_sinkage_m >= 0.035
    high_slip = measurement.scout_slip >= 0.25
    low_roughness = measurement.roughness_m < 0.025
    rough = measurement.roughness_m >= 0.045 or measurement.obstacle_height_m >= 0.055
    steep = abs(measurement.slope_long_deg) >= 12.0 or abs(measurement.slope_lat_deg) >= 10.0
    obstacle = measurement.obstacle_height_m >= 0.09 or measurement.gap_width_m >= 0.22

    granular = high_sinkage or (high_slip and low_roughness)

    strong_flags = sum(
        [
            granular and (measurement.scout_sinkage_m >= 0.055 or measurement.scout_slip >= 0.4),
            rough and measurement.roughness_m >= 0.065,
            steep and (abs(measurement.slope_long_deg) >= 20.0 or abs(measurement.slope_lat_deg) >= 16.0),
            obstacle,
        ]
    )

    if strong_flags >= 2:
        return TerrainClassification(
            "mixed",
            "multiple strong indicators are active: granular, rough/slope, or obstacle/gap",
            0.78,
        )
    if obstacle:
        return TerrainClassification("obstacle_step", "obstacle height or gap width exceeds the step/gap threshold", 0.82)
    if granular and steep:
        return TerrainClassification("granular_slope", "granular response combined with significant slope", 0.8)
    if granular:
        return TerrainClassification("granular", "sinkage is high, or slip is high while roughness is low", 0.78)
    if rough:
        return TerrainClassification("rocky_rough", "roughness or local obstacle height is elevated", 0.76)
    if steep:
        return TerrainClassification("rigid_slope", "slope is elevated without granular or obstacle indicators", 0.74)

    return TerrainClassification("rigid_flat", "low slope, low roughness, low sinkage, and low obstacle indicators", 0.72)


def _has_missing_numeric(measurement: ScoutMeasurement) -> bool:
    numeric_values = [
        measurement.slope_long_deg,
        measurement.slope_lat_deg,
        measurement.roughness_m,
        measurement.obstacle_height_m,
        measurement.gap_width_m,
        measurement.scout_slip,
        measurement.scout_sinkage_m,
        measurement.scout_wheel_torque_nm,
        measurement.scout_cot,
        measurement.vibration_rms_g,
    ]
    return any(value != value for value in numeric_values)

