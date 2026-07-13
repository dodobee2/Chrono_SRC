from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any
from uuid import uuid4

from .schemas import MainRoverConfig, ScoutMeasurement


def _require_positive(name: str, value: float) -> None:
    if value <= 0:
        raise ValueError(f"{name} must be positive")


def _require_nonnegative(name: str, value: float) -> None:
    if value < 0:
        raise ValueError(f"{name} must be nonnegative")


@dataclass(frozen=True)
class RoverSpec:
    """Replaceable rover model contract. All fields use SI units."""

    rover_id: str
    display_name: str
    mass_kg: float
    wheel_radius_m: float
    wheel_width_m: float
    wheelbase_m: float
    track_width_m: float
    cg_height_m: float
    ground_clearance_m: float
    driven_wheel_count: int
    max_wheel_torque_nm: float
    mu_eff: float
    crr: float
    model_uri: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.rover_id:
            raise ValueError("rover_id is required")
        for name in [
            "mass_kg",
            "wheel_radius_m",
            "wheel_width_m",
            "wheelbase_m",
            "track_width_m",
            "cg_height_m",
            "ground_clearance_m",
            "max_wheel_torque_nm",
        ]:
            _require_positive(name, float(getattr(self, name)))
        if self.driven_wheel_count <= 0:
            raise ValueError("driven_wheel_count must be positive")
        _require_positive("mu_eff", self.mu_eff)
        _require_nonnegative("crr", self.crr)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "RoverSpec":
        return cls(
            rover_id=str(data["rover_id"]),
            display_name=str(data.get("display_name") or data["rover_id"]),
            mass_kg=float(data["mass_kg"]),
            wheel_radius_m=float(data["wheel_radius_m"]),
            wheel_width_m=float(data["wheel_width_m"]),
            wheelbase_m=float(data["wheelbase_m"]),
            track_width_m=float(data["track_width_m"]),
            cg_height_m=float(data["cg_height_m"]),
            ground_clearance_m=float(data["ground_clearance_m"]),
            driven_wheel_count=int(data["driven_wheel_count"]),
            max_wheel_torque_nm=float(data["max_wheel_torque_nm"]),
            mu_eff=float(data["mu_eff"]),
            crr=float(data["crr"]),
            model_uri=str(data.get("model_uri", "") or ""),
            metadata=dict(data.get("metadata", {}) or {}),
        )

    def to_main_config(self) -> MainRoverConfig:
        return MainRoverConfig(
            mass_kg=self.mass_kg,
            wheel_radius_m=self.wheel_radius_m,
            wheel_width_m=self.wheel_width_m,
            wheelbase_m=self.wheelbase_m,
            track_width_m=self.track_width_m,
            cg_height_m=self.cg_height_m,
            ground_clearance_m=self.ground_clearance_m,
            driven_wheel_count=self.driven_wheel_count,
            max_wheel_torque_nm=self.max_wheel_torque_nm,
            mu_eff=self.mu_eff,
            crr=self.crr,
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ObstacleSpec:
    """Terrain obstacle primitive placeholder. All dimensions use meters."""

    obstacle_id: str
    kind: str
    x_m: float
    y_m: float
    height_m: float
    width_m: float = 0.0
    length_m: float = 0.0
    radius_m: float = 0.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.obstacle_id:
            raise ValueError("obstacle_id is required")
        if self.kind not in {"rock", "step", "gap", "ridge", "generic"}:
            raise ValueError("kind must be rock, step, gap, ridge, or generic")
        for name in ["height_m", "width_m", "length_m", "radius_m"]:
            _require_nonnegative(name, float(getattr(self, name)))

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "ObstacleSpec":
        return cls(
            obstacle_id=str(data["obstacle_id"]),
            kind=str(data.get("kind", "generic")),
            x_m=float(data.get("x_m", 0.0)),
            y_m=float(data.get("y_m", 0.0)),
            height_m=float(data.get("height_m", 0.0)),
            width_m=float(data.get("width_m", 0.0)),
            length_m=float(data.get("length_m", 0.0)),
            radius_m=float(data.get("radius_m", 0.0)),
            metadata=dict(data.get("metadata", {}) or {}),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TerrainScenario:
    """Replaceable terrain scenario contract. All numeric fields use SI units."""

    terrain_id: str
    display_name: str
    terrain_type: str
    surface_hint: str
    slope_long_deg: float
    slope_lat_deg: float
    roughness_m: float
    gap_width_m: float
    obstacles: list[ObstacleSpec] = field(default_factory=list)
    scout_response: dict[str, float] = field(default_factory=dict)
    patch_id: int = 1
    grid_x: int = 0
    grid_y: int = 0
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.terrain_id:
            raise ValueError("terrain_id is required")
        if self.terrain_type not in {"rigid", "rocky", "granular", "mixed", "unknown"}:
            raise ValueError("terrain_type must be rigid, rocky, granular, mixed, or unknown")
        _require_nonnegative("roughness_m", self.roughness_m)
        _require_nonnegative("gap_width_m", self.gap_width_m)
        if abs(self.slope_long_deg) > 89 or abs(self.slope_lat_deg) > 89:
            raise ValueError("slope angles must be within +/-89 degrees")
        for key in ["scout_slip", "scout_sinkage_m", "scout_wheel_torque_nm", "scout_cot", "vibration_rms_g"]:
            if key not in self.scout_response:
                raise ValueError(f"scout_response.{key} is required")
            _require_nonnegative(f"scout_response.{key}", float(self.scout_response[key]))

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "TerrainScenario":
        obstacles = [ObstacleSpec.from_mapping(item) for item in data.get("obstacles", []) or []]
        return cls(
            terrain_id=str(data["terrain_id"]),
            display_name=str(data.get("display_name") or data["terrain_id"]),
            terrain_type=str(data.get("terrain_type", "unknown")),
            surface_hint=str(data.get("surface_hint", "") or ""),
            slope_long_deg=float(data.get("slope_long_deg", 0.0)),
            slope_lat_deg=float(data.get("slope_lat_deg", 0.0)),
            roughness_m=float(data.get("roughness_m", 0.0)),
            gap_width_m=float(data.get("gap_width_m", 0.0)),
            obstacles=obstacles,
            scout_response={key: float(value) for key, value in dict(data.get("scout_response", {}) or {}).items()},
            patch_id=int(data.get("patch_id", 1)),
            grid_x=int(data.get("grid_x", 0)),
            grid_y=int(data.get("grid_y", 0)),
            metadata=dict(data.get("metadata", {}) or {}),
        )

    @property
    def max_obstacle_height_m(self) -> float:
        if not self.obstacles:
            return 0.0
        return max(obstacle.height_m for obstacle in self.obstacles)

    def to_scout_measurement(self) -> ScoutMeasurement:
        return ScoutMeasurement(
            patch_id=self.patch_id,
            grid_x=self.grid_x,
            grid_y=self.grid_y,
            slope_long_deg=self.slope_long_deg,
            slope_lat_deg=self.slope_lat_deg,
            roughness_m=self.roughness_m,
            obstacle_height_m=self.max_obstacle_height_m,
            gap_width_m=self.gap_width_m,
            scout_slip=float(self.scout_response["scout_slip"]),
            scout_sinkage_m=float(self.scout_response["scout_sinkage_m"]),
            scout_wheel_torque_nm=float(self.scout_response["scout_wheel_torque_nm"]),
            scout_cot=float(self.scout_response["scout_cot"]),
            vibration_rms_g=float(self.scout_response["vibration_rms_g"]),
            surface_hint=self.surface_hint,
        )

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["obstacles"] = [obstacle.to_dict() for obstacle in self.obstacles]
        return result


@dataclass(frozen=True)
class ControlProfile:
    """Open-loop control profile placeholder. All fields use SI units except steering degrees."""

    profile_id: str
    display_name: str
    target_speed_mps: float
    duration_s: float
    throttle: float
    steering_deg: float = 0.0
    drive_mode: str = "velocity_hold"
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.profile_id:
            raise ValueError("profile_id is required")
        _require_nonnegative("target_speed_mps", self.target_speed_mps)
        _require_positive("duration_s", self.duration_s)
        if not 0.0 <= self.throttle <= 1.0:
            raise ValueError("throttle must be in 0..1")
        if abs(self.steering_deg) > 90:
            raise ValueError("steering_deg must be within +/-90")

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "ControlProfile":
        return cls(
            profile_id=str(data["profile_id"]),
            display_name=str(data.get("display_name") or data["profile_id"]),
            target_speed_mps=float(data["target_speed_mps"]),
            duration_s=float(data["duration_s"]),
            throttle=float(data["throttle"]),
            steering_deg=float(data.get("steering_deg", 0.0)),
            drive_mode=str(data.get("drive_mode", "velocity_hold")),
            metadata=dict(data.get("metadata", {}) or {}),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SimulationResult:
    """Common backend output consumed by risk fusion, UI, and future planners."""

    experiment_id: str
    backend_name: str
    rover_id: str
    terrain_id: str
    control_profile_id: str
    status: str
    started_at_utc: str
    duration_s: float
    metrics: dict[str, float]
    risk_components: dict[str, float]
    final_risk: float
    grade: str
    hard_failure_reasons: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    artifacts: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.status not in {"ok", "mock", "not_implemented", "failed"}:
            raise ValueError("status must be ok, mock, not_implemented, or failed")
        _require_nonnegative("duration_s", self.duration_s)
        if not 0.0 <= self.final_risk <= 1.0:
            raise ValueError("final_risk must be in 0..1")
        if self.grade not in {"Safe", "Caution", "Risk", "Unknown"}:
            raise ValueError("grade must be Safe, Caution, Risk, or Unknown")

    @classmethod
    def new(
        cls,
        backend_name: str,
        rover_id: str,
        terrain_id: str,
        control_profile_id: str,
        status: str,
        duration_s: float,
        metrics: dict[str, float],
        risk_components: dict[str, float],
        final_risk: float,
        grade: str,
        hard_failure_reasons: list[str] | None = None,
        notes: list[str] | None = None,
        artifacts: dict[str, str] | None = None,
    ) -> "SimulationResult":
        return cls(
            experiment_id=f"exp_{uuid4().hex[:12]}",
            backend_name=backend_name,
            rover_id=rover_id,
            terrain_id=terrain_id,
            control_profile_id=control_profile_id,
            status=status,
            started_at_utc=datetime.now(timezone.utc).isoformat(),
            duration_s=duration_s,
            metrics=metrics,
            risk_components=risk_components,
            final_risk=final_risk,
            grade=grade,
            hard_failure_reasons=hard_failure_reasons or [],
            notes=notes or [],
            artifacts=artifacts or {},
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

