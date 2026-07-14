"""Baselines for predicting main_v01's response from scout_v01's measured response.

Per the pilot's own ground rule: do not oversell a "prediction model" this
early. identity_baseline and slope_only are intentionally weak strawmen.
slope_and_soil reuses src/mobility_physics.py's existing
predict_main_slip/predict_main_sinkage_m rather than inventing new scaling
math, but note its real limitation: those functions scale slip/sinkage by
wheel geometry and axle load pressure only -- they do not consume Bekker
Kphi/Kc/n at all, so "soil" only enters this baseline's slip/sinkage output
indirectly (through whichever RunSummary numbers were actually measured on
that soil), not through the physics formula itself. Soil only directly
affects the torque/energy prediction, via the crr term in
required_traction_force_n. user_formula is a placeholder until a real
calibration pass exists -- it must stay NOT_CONFIGURED, not a guessed
formula.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import radians, tan
from typing import Any

from ...integration_schemas import RoverSpec
from ...mobility_physics import G_MPS2, predict_main_sinkage_m, predict_main_slip, required_traction_force_n
from ...schemas import MainRoverConfig, ScoutReferenceConfig
from .metrics import RunSummary


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


def identity_baseline(scout_summary: RunSummary) -> PredictionResult:
    """Dumb baseline: predict main's response is identical to scout's."""
    return PredictionResult(
        predictor_name="identity_baseline",
        status="OK",
        metrics={
            "mean_slip": scout_summary.mean_slip,
            "mean_sinkage_m": scout_summary.mean_sinkage_m or 0.0,
            "mean_wheel_torque_nm": scout_summary.mean_wheel_torque_nm,
            "energy_j": scout_summary.energy_j,
            "mean_speed_mps": scout_summary.mean_speed_mps,
        },
        notes=["Copies scout metrics verbatim onto main; ignores every difference between the two rovers."],
    )


def slope_only(
    scout_summary: RunSummary,
    scout_rover: RoverSpec,
    main_rover: RoverSpec,
    slope_deg: float,
) -> PredictionResult:
    """Scales torque/energy by the slope+mass force-balance ratio (crr=0); slip/sinkage copied from scout unmodified."""
    scout_force = required_traction_force_n(slope_deg, _rover_config(scout_rover, mu_eff=1.0, crr=0.0))
    main_force = required_traction_force_n(slope_deg, _rover_config(main_rover, mu_eff=1.0, crr=0.0))
    ratio = main_force / max(scout_force, 1e-9)
    return PredictionResult(
        predictor_name="slope_only",
        status="OK",
        metrics={
            "mean_slip": scout_summary.mean_slip,
            "mean_sinkage_m": scout_summary.mean_sinkage_m or 0.0,
            "mean_wheel_torque_nm": scout_summary.mean_wheel_torque_nm * ratio,
            "energy_j": scout_summary.energy_j * ratio,
            "mean_speed_mps": scout_summary.mean_speed_mps,
        },
        notes=[
            "Scales scout torque/energy by a mass*sin(slope) force-balance ratio only (crr=0, no soil).",
            "mean_slip/mean_sinkage_m are copied unmodified from scout -- this baseline has no soil model.",
        ],
    )


def slope_and_soil(
    scout_summary: RunSummary,
    scout_rover: RoverSpec,
    main_rover: RoverSpec,
    slope_deg: float,
    soil_mohr_friction_angle_deg: float,
    soil_janosi_shear_m: float,
) -> PredictionResult:
    """Reuses src/mobility_physics.py's predict_main_slip/predict_main_sinkage_m.

    mu_eff is approximated as tan(mohr_friction_angle) and crr as a fixed
    multiple of janosi_shear_m -- both are standard terramechanics proxies,
    not measured values. See TerrainMaterialSpec.scm_parameters for the
    source numbers behind soil_mohr_friction_angle_deg/soil_janosi_shear_m.
    """
    mu_eff = max(0.05, tan(radians(soil_mohr_friction_angle_deg)))
    crr = max(0.01, soil_janosi_shear_m * 2.0)

    main_config = _rover_config(main_rover, mu_eff=mu_eff, crr=crr)
    scout_reference = _scout_reference(scout_rover)

    predicted_slip = predict_main_slip(scout_summary.mean_slip, main_config, scout_reference)
    predicted_sinkage = predict_main_sinkage_m(scout_summary.mean_sinkage_m or 0.0, main_config, scout_reference)
    scout_force = required_traction_force_n(slope_deg, _rover_config(scout_rover, mu_eff=mu_eff, crr=crr))
    main_force = required_traction_force_n(slope_deg, main_config)
    ratio = main_force / max(scout_force, 1e-9)

    notes = [
        "Reuses src/mobility_physics.py::predict_main_slip/predict_main_sinkage_m (geometry/pressure scaling).",
        "Soil (Bekker Kphi/Kc) is NOT consumed by predict_main_slip/sinkage -- soil only enters this "
        "baseline's torque/energy prediction via the crr term.",
    ]
    if scout_summary.mean_sinkage_m is None:
        notes.append("scout mean_sinkage_m was unavailable; treated as 0.0.")
    return PredictionResult(
        predictor_name="slope_and_soil",
        status="OK",
        metrics={
            "mean_slip": predicted_slip,
            "mean_sinkage_m": predicted_sinkage,
            "mean_wheel_torque_nm": scout_summary.mean_wheel_torque_nm * ratio,
            "energy_j": scout_summary.energy_j * ratio,
            "mean_speed_mps": scout_summary.mean_speed_mps,
        },
        notes=notes,
    )


def user_formula(*_args: Any, **_kwargs: Any) -> PredictionResult:
    """Placeholder for a calibrated transfer formula (PDF section 05, Chrono Implementation).

    Deliberately returns NOT_CONFIGURED rather than a fabricated formula --
    do not fill this in with guessed coefficients.
    """
    return PredictionResult(
        predictor_name="user_formula",
        status="NOT_CONFIGURED",
        metrics=None,
        notes=[
            "No calibrated transfer formula yet.",
            "Requires a real calibration pass (CMA-ES/Bayesian, per PDF section 05) against measured data.",
        ],
    )
