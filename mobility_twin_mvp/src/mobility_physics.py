from __future__ import annotations

from dataclasses import dataclass
from math import atan, cos, degrees, radians, sin, sqrt

import numpy as np

from .schemas import MainRoverConfig, ScoutMeasurement, ScoutReferenceConfig


G_MPS2 = 9.80665


@dataclass(frozen=True)
class MobilityPhysicsResult:
    f_req_n: float
    f_torque_n: float
    f_friction_n: float
    f_avail_n: float
    traction_margin_n: float
    traction_margin_ratio: float
    beta_crit_deg: float
    tipover_margin_deg: float
    obstacle_ratio: float
    gap_ratio: float
    clearance_ratio: float
    pressure_ratio: float
    predicted_main_sinkage_m: float
    predicted_main_slip: float


def required_traction_force_n(slope_long_deg: float, config: MainRoverConfig) -> float:
    """Return uphill force demand in N; alpha uses absolute longitudinal slope in degrees."""
    alpha = radians(abs(slope_long_deg))
    return config.mass_kg * G_MPS2 * sin(alpha) + config.crr * config.mass_kg * G_MPS2 * cos(alpha)


def torque_limited_force_n(config: MainRoverConfig) -> float:
    """Return propulsion force in N from driven wheel torque and wheel radius in meters."""
    return config.driven_wheel_count * config.max_wheel_torque_nm / config.wheel_radius_m


def friction_limited_force_n(slope_long_deg: float, config: MainRoverConfig) -> float:
    """Return traction force in N limited by an effective friction coefficient."""
    alpha = radians(abs(slope_long_deg))
    return config.mu_eff * config.mass_kg * G_MPS2 * cos(alpha)


def available_traction_force_n(slope_long_deg: float, config: MainRoverConfig) -> float:
    """Return usable force in N as the minimum of torque and friction limits."""
    return min(torque_limited_force_n(config), friction_limited_force_n(slope_long_deg, config))


def beta_crit_deg(config: MainRoverConfig) -> float:
    """Return static lateral tipover threshold angle in degrees."""
    return degrees(atan((config.track_width_m / 2.0) / config.cg_height_m))


def obstacle_ratio(obstacle_height_m: float, config: MainRoverConfig) -> float:
    """Return obstacle height divided by main-rover wheel radius."""
    return obstacle_height_m / config.wheel_radius_m


def gap_ratio(gap_width_m: float, config: MainRoverConfig) -> float:
    """Return gap width divided by main-rover wheel diameter."""
    return gap_width_m / (2.0 * config.wheel_radius_m)


def clearance_ratio(obstacle_height_m: float, config: MainRoverConfig) -> float:
    """Return obstacle height divided by main-rover ground clearance."""
    return obstacle_height_m / config.ground_clearance_m


def pressure_ratio(config: MainRoverConfig, scout_reference: ScoutReferenceConfig) -> float:
    """Return MVP pressure scaling ratio.

    This intentionally follows the requested heuristic: main mass per driven wheel
    divided by the reference scout wheel load scalar in N.
    """
    main_wheel_load_n = config.mass_kg * G_MPS2 / config.driven_wheel_count
    return main_wheel_load_n / scout_reference.wheel_load_n


def predict_main_sinkage_m(
    scout_sinkage_m: float,
    config: MainRoverConfig,
    scout_reference: ScoutReferenceConfig,
) -> float:
    """Scale scout sinkage to the main rover with MVP pressure and wheel-width heuristics."""
    pr = max(pressure_ratio(config, scout_reference), 0.1)
    return scout_sinkage_m * sqrt(pr) * sqrt(scout_reference.wheel_width_m / config.wheel_width_m)


def predict_main_slip(
    scout_slip: float,
    config: MainRoverConfig,
    scout_reference: ScoutReferenceConfig,
) -> float:
    """Scale scout slip to the main rover and clip it to the physical 0..1 range."""
    pr = max(pressure_ratio(config, scout_reference), 0.1)
    scaled = scout_slip * sqrt(pr) * sqrt(scout_reference.wheel_radius_m / config.wheel_radius_m)
    return float(np.clip(scaled, 0.0, 1.0))


def calculate_mobility_physics(
    measurement: ScoutMeasurement,
    config: MainRoverConfig,
    scout_reference: ScoutReferenceConfig,
) -> MobilityPhysicsResult:
    """Calculate per-patch mobility terms for the main rover."""
    f_req = required_traction_force_n(measurement.slope_long_deg, config)
    f_torque = torque_limited_force_n(config)
    f_friction = friction_limited_force_n(measurement.slope_long_deg, config)
    f_avail = min(f_torque, f_friction)
    traction_margin = f_avail - f_req
    beta = beta_crit_deg(config)
    obs_ratio = obstacle_ratio(measurement.obstacle_height_m, config)
    gp_ratio = gap_ratio(measurement.gap_width_m, config)
    clr_ratio = clearance_ratio(measurement.obstacle_height_m, config)
    pr = pressure_ratio(config, scout_reference)
    sinkage = predict_main_sinkage_m(measurement.scout_sinkage_m, config, scout_reference)
    slip = predict_main_slip(measurement.scout_slip, config, scout_reference)

    return MobilityPhysicsResult(
        f_req_n=f_req,
        f_torque_n=f_torque,
        f_friction_n=f_friction,
        f_avail_n=f_avail,
        traction_margin_n=traction_margin,
        traction_margin_ratio=traction_margin / max(f_req, 1.0),
        beta_crit_deg=beta,
        tipover_margin_deg=beta - abs(measurement.slope_lat_deg),
        obstacle_ratio=obs_ratio,
        gap_ratio=gp_ratio,
        clearance_ratio=clr_ratio,
        pressure_ratio=pr,
        predicted_main_sinkage_m=sinkage,
        predicted_main_slip=slip,
    )

