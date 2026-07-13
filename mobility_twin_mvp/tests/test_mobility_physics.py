from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mobility_physics import beta_crit_deg, calculate_mobility_physics, required_traction_force_n
from src.risk_fusion import analyze_dataframe, analyze_patch, calculate_risk_components
from src.sample_generator import generate_sample_patches
from src.schemas import DEFAULT_SCOUT_REFERENCE, MainRoverConfig, ScoutMeasurement
from src.terrain_classifier import classify_terrain


def make_measurement(**overrides: object) -> ScoutMeasurement:
    data = {
        "patch_id": 1,
        "grid_x": 0,
        "grid_y": 0,
        "slope_long_deg": 0.0,
        "slope_lat_deg": 0.0,
        "roughness_m": 0.01,
        "obstacle_height_m": 0.01,
        "gap_width_m": 0.0,
        "scout_slip": 0.05,
        "scout_sinkage_m": 0.005,
        "scout_wheel_torque_nm": 0.5,
        "scout_cot": 0.4,
        "vibration_rms_g": 0.05,
        "surface_hint": "",
    }
    data.update(overrides)
    return ScoutMeasurement.from_mapping(data)


def test_required_force_increases_with_slope() -> None:
    config = MainRoverConfig()
    assert required_traction_force_n(18.0, config) > required_traction_force_n(5.0, config)


def test_available_force_does_not_decrease_when_max_torque_increases() -> None:
    measurement = make_measurement(slope_long_deg=8.0)
    low_torque = MainRoverConfig(max_wheel_torque_nm=6.0)
    high_torque = MainRoverConfig(max_wheel_torque_nm=20.0)
    low = calculate_mobility_physics(measurement, low_torque, DEFAULT_SCOUT_REFERENCE).f_avail_n
    high = calculate_mobility_physics(measurement, high_torque, DEFAULT_SCOUT_REFERENCE).f_avail_n
    assert high >= low


def test_beta_crit_decreases_when_cg_height_increases() -> None:
    low_cg = MainRoverConfig(cg_height_m=0.18)
    high_cg = MainRoverConfig(cg_height_m=0.35)
    assert beta_crit_deg(high_cg) < beta_crit_deg(low_cg)


def test_obstacle_risk_increases_with_obstacle_ratio() -> None:
    config = MainRoverConfig()
    low = make_measurement(obstacle_height_m=0.02)
    high = make_measurement(obstacle_height_m=0.10)
    low_physics = calculate_mobility_physics(low, config, DEFAULT_SCOUT_REFERENCE)
    high_physics = calculate_mobility_physics(high, config, DEFAULT_SCOUT_REFERENCE)
    low_risk = calculate_risk_components(low, classify_terrain(low), low_physics)["obstacle"]
    high_risk = calculate_risk_components(high, classify_terrain(high), high_physics)["obstacle"]
    assert high_risk > low_risk


def test_hard_failure_returns_risk_grade() -> None:
    measurement = make_measurement(obstacle_height_m=0.15)
    result = analyze_patch(measurement, MainRoverConfig(), DEFAULT_SCOUT_REFERENCE)
    assert result.grade == "Risk"
    assert result.hard_failure_reasons


def test_all_final_risks_are_in_unit_range() -> None:
    results = analyze_dataframe(generate_sample_patches(), MainRoverConfig(), DEFAULT_SCOUT_REFERENCE)
    assert ((results["final_risk"] >= 0.0) & (results["final_risk"] <= 1.0)).all()
    assert {"Safe", "Caution", "Risk"}.issubset(set(results["grade"]))

