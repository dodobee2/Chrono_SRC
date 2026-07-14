"""Baselines for predicting main_v01's rigid-terrain response from scout_v01's.

Interpretation note on the pilot spec's "예측 인터페이스:
predict_main_from_scout(scout_metrics, scout_spec, main_spec, terrain_context,
predictor_config)": implemented as a common signature shared by all four
predictors below, plus one dispatcher function of that name that selects
among them by predictor_name -- a single function can't itself "be" four
different comparable baselines, so the runner/evaluator loop calls the
dispatcher once per predictor per condition.

slope_only and terrain_only reuse src/mobility_physics.py's predict_main_slip
and required_traction_force_n rather than inventing new scaling math (same
approach as src/experiments/scm_pilot/predictor.py). terrain_only differs
from slope_only only in using the terrain's actual friction_nominal /
rolling_resistance_nominal as mu_eff/crr instead of ignoring them (crr=0,
mu_eff=1). Sinkage is not part of this pilot -- rigid terrain has no sinkage
model, and it isn't in the instructed metrics list.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from ...integration_schemas import RoverSpec
from ...mobility_physics import G_MPS2, predict_main_slip, required_traction_force_n
from ...schemas import MainRoverConfig, ScoutReferenceConfig
from .metrics import RunSummary
from .scenario import TerrainContext

COMPARED_METRICS = ("mean_slip", "mean_wheel_torque_nm", "energy_j", "mean_speed_mps", "distance_m")


@dataclass(frozen=True)
class PredictionResult:
    predictor_name: str
    status: str  # "OK" | "NOT_CONFIGURED"
    metrics: dict[str, float] | None
    notes: list[str]


def _rover_config(rover: RoverSpec, mu_eff: float, crr: float) -> MainRoverConfig:
    return MainRoverConfig(
        mass_kg=rover.mass_kg,
        wheel_radius_m=rover.wheel_radius_m,
        wheel_width_m=rover.wheel_width_m,
        wheelbase_m=rover.wheelbase_m,
        track_width_m=rover.track_width_m,
        cg_height_m=rover.cg_height_m,
        ground_clearance_m=rover.ground_clearance_m,
        driven_wheel_count=rover.driven_wheel_count,
        max_wheel_torque_nm=rover.max_wheel_torque_nm,
        mu_eff=mu_eff,
        crr=crr,
    )


def _scout_reference(scout_rover: RoverSpec) -> ScoutReferenceConfig:
    wheel_load_n = scout_rover.mass_kg * G_MPS2 / max(scout_rover.driven_wheel_count, 1)
    return ScoutReferenceConfig(
        mass_kg=scout_rover.mass_kg,
        wheel_radius_m=scout_rover.wheel_radius_m,
        wheel_width_m=scout_rover.wheel_width_m,
        wheel_load_n=wheel_load_n,
    )


def _copied_metrics(scout_metrics: RunSummary) -> dict[str, float]:
    return {
        "mean_slip": scout_metrics.mean_slip,
        "mean_wheel_torque_nm": scout_metrics.mean_wheel_torque_nm,
        "energy_j": scout_metrics.energy_j,
        "mean_speed_mps": scout_metrics.mean_speed_mps,
        "distance_m": scout_metrics.distance_m,
    }


def identity_baseline(
    scout_metrics: RunSummary,
    scout_spec: RoverSpec,
    main_spec: RoverSpec,
    terrain_context: TerrainContext,
    predictor_config: dict[str, Any] | None = None,
) -> PredictionResult:
    return PredictionResult(
        predictor_name="identity_baseline",
        status="OK",
        metrics=_copied_metrics(scout_metrics),
        notes=["Copies scout metrics verbatim onto main; ignores every difference between the two rovers and the terrain."],
    )


def slope_only(
    scout_metrics: RunSummary,
    scout_spec: RoverSpec,
    main_spec: RoverSpec,
    terrain_context: TerrainContext,
    predictor_config: dict[str, Any] | None = None,
) -> PredictionResult:
    scout_force = required_traction_force_n(terrain_context.slope_deg, _rover_config(scout_spec, mu_eff=1.0, crr=0.0))
    main_force = required_traction_force_n(terrain_context.slope_deg, _rover_config(main_spec, mu_eff=1.0, crr=0.0))
    ratio = main_force / max(scout_force, 1e-9)
    metrics = _copied_metrics(scout_metrics)
    metrics["mean_wheel_torque_nm"] *= ratio
    metrics["energy_j"] *= ratio
    return PredictionResult(
        predictor_name="slope_only",
        status="OK",
        metrics=metrics,
        notes=[
            "Scales scout torque/energy by a mass*sin(slope) force-balance ratio only "
            "(crr=0, friction and obstacle ignored).",
            "mean_slip/distance/speed are copied unmodified from scout.",
        ],
    )


def terrain_only(
    scout_metrics: RunSummary,
    scout_spec: RoverSpec,
    main_spec: RoverSpec,
    terrain_context: TerrainContext,
    predictor_config: dict[str, Any] | None = None,
) -> PredictionResult:
    mu_eff = max(0.05, terrain_context.friction_nominal)
    crr = max(0.0, terrain_context.rolling_resistance_nominal)

    main_config = _rover_config(main_spec, mu_eff=mu_eff, crr=crr)
    scout_reference = _scout_reference(scout_spec)
    predicted_slip = predict_main_slip(scout_metrics.mean_slip, main_config, scout_reference)

    scout_force = required_traction_force_n(terrain_context.slope_deg, _rover_config(scout_spec, mu_eff=mu_eff, crr=crr))
    main_force = required_traction_force_n(terrain_context.slope_deg, main_config)
    ratio = main_force / max(scout_force, 1e-9)

    notes = [
        "Reuses src/mobility_physics.py::predict_main_slip and required_traction_force_n.",
        "friction_nominal/rolling_resistance_nominal come directly from the terrain material "
        "(unlike SCM soil, rigid materials have these fields populated -- no proxy derivation needed).",
    ]
    if terrain_context.obstacle_height_m > 0:
        notes.append(
            f"obstacle_height_m={terrain_context.obstacle_height_m:.3f} is NOT modeled in this baseline's "
            "torque/slip scaling -- only slope and friction are. Treat obstacle-condition predictions as weaker."
        )
    metrics = _copied_metrics(scout_metrics)
    metrics["mean_slip"] = predicted_slip
    metrics["mean_wheel_torque_nm"] *= ratio
    metrics["energy_j"] *= ratio
    return PredictionResult(predictor_name="terrain_only", status="OK", metrics=metrics, notes=notes)


def user_formula(
    scout_metrics: RunSummary,
    scout_spec: RoverSpec,
    main_spec: RoverSpec,
    terrain_context: TerrainContext,
    predictor_config: dict[str, Any] | None = None,
) -> PredictionResult:
    """Placeholder for a user-supplied calibrated formula. NOT_CONFIGURED until
    predictor_config supplies real coefficients -- never fill this in with guessed values.

    predictor_config, when provided, is expected as:
        {"formula": "linear_mass_scale_v1",
         "coefficients": {"slip_mass_scale": <float>, "torque_mass_scale": <float>}}
    Any other/missing config returns NOT_CONFIGURED.
    """
    if not predictor_config or predictor_config.get("formula") != "linear_mass_scale_v1":
        return PredictionResult(
            predictor_name="user_formula",
            status="NOT_CONFIGURED",
            metrics=None,
            notes=[
                "No calibrated formula configured.",
                "Pass predictor_config={'formula': 'linear_mass_scale_v1', 'coefficients': {...}} to enable.",
            ],
        )
    coefficients = dict(predictor_config.get("coefficients", {}) or {})
    mass_ratio = main_spec.mass_kg / max(scout_spec.mass_kg, 1e-9)
    slip_scale = float(coefficients.get("slip_mass_scale", 1.0))
    torque_scale = float(coefficients.get("torque_mass_scale", 1.0))
    metrics = _copied_metrics(scout_metrics)
    metrics["mean_slip"] = min(1.0, max(0.0, scout_metrics.mean_slip * (mass_ratio**slip_scale)))
    metrics["mean_wheel_torque_nm"] *= mass_ratio**torque_scale
    metrics["energy_j"] *= mass_ratio**torque_scale
    return PredictionResult(
        predictor_name="user_formula",
        status="OK",
        metrics=metrics,
        notes=["User-supplied linear_mass_scale_v1 formula applied; not calibrated against measured data unless predictor_config says so."],
    )


PREDICTORS: dict[str, Callable[..., PredictionResult]] = {
    "identity_baseline": identity_baseline,
    "slope_only": slope_only,
    "terrain_only": terrain_only,
    "user_formula": user_formula,
}


def predict_main_from_scout(
    predictor_name: str,
    scout_metrics: RunSummary,
    scout_spec: RoverSpec,
    main_spec: RoverSpec,
    terrain_context: TerrainContext,
    predictor_config: dict[str, Any] | None = None,
) -> PredictionResult:
    if predictor_name not in PREDICTORS:
        raise ValueError(f"unknown predictor_name {predictor_name!r}; expected one of {sorted(PREDICTORS)}")
    return PREDICTORS[predictor_name](scout_metrics, scout_spec, main_spec, terrain_context, predictor_config)
