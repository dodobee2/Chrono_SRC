from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ScoutMeasurement:
    patch_id: int
    grid_x: int
    grid_y: int
    slope_long_deg: float
    slope_lat_deg: float
    roughness_m: float
    obstacle_height_m: float
    gap_width_m: float
    scout_slip: float
    scout_sinkage_m: float
    scout_wheel_torque_nm: float
    scout_cot: float
    vibration_rms_g: float
    surface_hint: str = ""

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "ScoutMeasurement":
        return cls(
            patch_id=int(data["patch_id"]),
            grid_x=int(data["grid_x"]),
            grid_y=int(data["grid_y"]),
            slope_long_deg=float(data["slope_long_deg"]),
            slope_lat_deg=float(data["slope_lat_deg"]),
            roughness_m=float(data["roughness_m"]),
            obstacle_height_m=float(data["obstacle_height_m"]),
            gap_width_m=float(data["gap_width_m"]),
            scout_slip=float(data["scout_slip"]),
            scout_sinkage_m=float(data["scout_sinkage_m"]),
            scout_wheel_torque_nm=float(data["scout_wheel_torque_nm"]),
            scout_cot=float(data["scout_cot"]),
            vibration_rms_g=float(data["vibration_rms_g"]),
            surface_hint=str(data.get("surface_hint", "") or "").strip(),
        )


@dataclass(frozen=True)
class MainRoverConfig:
    mass_kg: float = 48.0
    wheel_radius_m: float = 0.16
    wheel_width_m: float = 0.08
    wheelbase_m: float = 0.55
    track_width_m: float = 0.45
    cg_height_m: float = 0.22
    ground_clearance_m: float = 0.12
    driven_wheel_count: int = 4
    max_wheel_torque_nm: float = 14.0
    mu_eff: float = 0.65
    crr: float = 0.08


@dataclass(frozen=True)
class ScoutReferenceConfig:
    mass_kg: float = 4.0
    wheel_radius_m: float = 0.055
    wheel_width_m: float = 0.035
    wheel_load_n: float = 10.0


DEFAULT_MAIN_ROVER = MainRoverConfig()
DEFAULT_SCOUT_REFERENCE = ScoutReferenceConfig()


REQUIRED_MEASUREMENT_COLUMNS = [
    "patch_id",
    "grid_x",
    "grid_y",
    "slope_long_deg",
    "slope_lat_deg",
    "roughness_m",
    "obstacle_height_m",
    "gap_width_m",
    "scout_slip",
    "scout_sinkage_m",
    "scout_wheel_torque_nm",
    "scout_cot",
    "vibration_rms_g",
    "surface_hint",
]

