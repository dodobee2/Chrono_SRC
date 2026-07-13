from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any
from uuid import uuid4
import warnings

from .schemas import MainRoverConfig, ScoutMeasurement


OBSERVATION_STATES = {"UNKNOWN", "GEOMETRY_ONLY", "SCOUT_OBSERVED", "SCOUT_TRAVERSED", "MAIN_EVALUATED"}
SOURCE_TYPES = {"mock", "simulation", "sensor_log", "manual"}
MODEL_STATUSES = {"heuristic", "mock", "chrono_smoke", "chrono_physics"}
EVALUATION_STATES = {"NOT_EVALUATED", "REFERENCE_ONLY", "EVALUATED"}


def _require_positive(name: str, value: float) -> None:
    if value <= 0:
        raise ValueError(f"{name} must be positive")


def _require_nonnegative(name: str, value: float) -> None:
    if value < 0:
        raise ValueError(f"{name} must be nonnegative")


def _require_confidence(name: str, value: float) -> None:
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be in 0..1")


def _tuple3(value: Any, default: tuple[float, float, float] | None = None) -> tuple[float, float, float]:
    if value is None:
        if default is None:
            raise ValueError("tuple3 value is required")
        return default
    if len(value) != 3:
        raise ValueError("tuple3 value must contain exactly three numbers")
    return (float(value[0]), float(value[1]), float(value[2]))


@dataclass(frozen=True)
class Pose3D:
    xyz_m: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]

    @classmethod
    def from_mapping(cls, data: dict[str, Any] | None) -> "Pose3D":
        payload = data or {}
        return cls(
            xyz_m=_tuple3(payload.get("xyz_m"), (0.0, 0.0, 0.0)),
            rpy_deg=_tuple3(payload.get("rpy_deg"), (0.0, 0.0, 0.0)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {"xyz_m": list(self.xyz_m), "rpy_deg": list(self.rpy_deg)}


@dataclass(frozen=True)
class TerrainGeometrySpec:
    source_type: str
    asset_uri: str | None
    factory_uri: str | None
    scale_xyz: tuple[float, float, float]
    frame_id: str
    coordinate_convention: str
    origin_pose: Pose3D
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.source_type not in {"procedural", "mesh", "heightmap", "code_factory", "unknown"}:
            raise ValueError("source_type must be procedural, mesh, heightmap, code_factory, or unknown")
        if not self.frame_id:
            raise ValueError("frame_id is required")
        if not self.coordinate_convention:
            raise ValueError("coordinate_convention is required")
        for index, value in enumerate(self.scale_xyz):
            _require_positive(f"scale_xyz[{index}]", value)

    @classmethod
    def from_mapping(cls, data: dict[str, Any] | None) -> "TerrainGeometrySpec":
        payload = data or {}
        return cls(
            source_type=str(payload.get("source_type", "unknown")),
            asset_uri=payload.get("asset_uri"),
            factory_uri=payload.get("factory_uri"),
            scale_xyz=_tuple3(payload.get("scale_xyz"), (1.0, 1.0, 1.0)),
            frame_id=str(payload.get("frame_id", "terrain")),
            coordinate_convention=str(payload.get("coordinate_convention", "Chrono_XForward_YLeft_ZUp")),
            origin_pose=Pose3D.from_mapping(payload.get("origin_pose")),
            metadata=dict(payload.get("metadata", {}) or {}),
        )

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["scale_xyz"] = list(self.scale_xyz)
        result["origin_pose"] = self.origin_pose.to_dict()
        return result


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
    wheel_material_id: str
    wheel_contact_model: str
    fallback_mu_eff: float | None = None
    fallback_crr: float | None = None
    model_uri: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.rover_id:
            raise ValueError("rover_id is required")
        if not self.wheel_material_id:
            raise ValueError("wheel_material_id is required")
        if not self.wheel_contact_model:
            raise ValueError("wheel_contact_model is required")
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
        if self.fallback_mu_eff is not None:
            _require_positive("fallback_mu_eff", self.fallback_mu_eff)
        if self.fallback_crr is not None:
            _require_nonnegative("fallback_crr", self.fallback_crr)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "RoverSpec":
        fallback_mu = data.get("fallback_mu_eff", data.get("mu_eff"))
        fallback_crr = data.get("fallback_crr", data.get("crr"))
        if "mu_eff" in data or "crr" in data:
            warnings.warn(
                "RoverSpec.mu_eff/crr are deprecated; migrated to fallback_mu_eff/fallback_crr.",
                DeprecationWarning,
                stacklevel=2,
            )
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
            wheel_material_id=str(data.get("wheel_material_id", "wheel_material_unknown")),
            wheel_contact_model=str(data.get("wheel_contact_model", "unknown")),
            fallback_mu_eff=None if fallback_mu is None else float(fallback_mu),
            fallback_crr=None if fallback_crr is None else float(fallback_crr),
            model_uri=str(data.get("model_uri", "") or ""),
            metadata=dict(data.get("metadata", {}) or {}),
        )

    def to_main_config(self, mu_eff: float | None = None, crr: float | None = None) -> MainRoverConfig:
        resolved_mu = self.fallback_mu_eff if mu_eff is None else mu_eff
        resolved_crr = self.fallback_crr if crr is None else crr
        if resolved_mu is None or resolved_crr is None:
            raise ValueError("effective mu_eff and crr are required for heuristic evaluation")
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
            mu_eff=resolved_mu,
            crr=resolved_crr,
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TerrainMaterialSpec:
    material_id: str
    model_type: str
    friction_nominal: float | None
    rolling_resistance_nominal: float | None
    restitution: float | None
    scm_parameters: dict[str, Any] = field(default_factory=dict)
    parameter_source: str = "assumed"
    confidence: float = 0.2
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.material_id:
            raise ValueError("material_id is required")
        if self.model_type not in {"rigid", "scm", "dem", "unknown"}:
            raise ValueError("model_type must be rigid, scm, dem, or unknown")
        if self.parameter_source not in {"assumed", "preset", "measured", "calibrated"}:
            raise ValueError("parameter_source must be assumed, preset, measured, or calibrated")
        _require_confidence("confidence", self.confidence)
        for name in ["friction_nominal", "rolling_resistance_nominal", "restitution"]:
            value = getattr(self, name)
            if value is not None:
                _require_nonnegative(name, value)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "TerrainMaterialSpec":
        return cls(
            material_id=str(data["material_id"]),
            model_type=str(data.get("model_type", "unknown")),
            friction_nominal=None if data.get("friction_nominal") is None else float(data["friction_nominal"]),
            rolling_resistance_nominal=None
            if data.get("rolling_resistance_nominal") is None
            else float(data["rolling_resistance_nominal"]),
            restitution=None if data.get("restitution") is None else float(data["restitution"]),
            scm_parameters=dict(data.get("scm_parameters", {}) or {}),
            parameter_source=str(data.get("parameter_source", "assumed")),
            confidence=float(data.get("confidence", 0.2)),
            metadata=dict(data.get("metadata", {}) or {}),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ContactPairSpec:
    contact_pair_id: str
    wheel_material_id: str
    terrain_material_id: str
    mu_eff: float | None
    crr_eff: float | None
    source: str
    confidence: float
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.contact_pair_id:
            raise ValueError("contact_pair_id is required")
        if not self.wheel_material_id:
            raise ValueError("wheel_material_id is required")
        if not self.terrain_material_id:
            raise ValueError("terrain_material_id is required")
        if self.source not in {"assumed", "preset", "measured", "calibrated", "unknown"}:
            raise ValueError("source must be assumed, preset, measured, calibrated, or unknown")
        _require_confidence("confidence", self.confidence)
        if self.mu_eff is not None:
            _require_positive("mu_eff", self.mu_eff)
        if self.crr_eff is not None:
            _require_nonnegative("crr_eff", self.crr_eff)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "ContactPairSpec":
        return cls(
            contact_pair_id=str(data["contact_pair_id"]),
            wheel_material_id=str(data["wheel_material_id"]),
            terrain_material_id=str(data["terrain_material_id"]),
            mu_eff=None if data.get("mu_eff") is None else float(data["mu_eff"]),
            crr_eff=None if data.get("crr_eff") is None else float(data["crr_eff"]),
            source=str(data.get("source", "unknown")),
            confidence=float(data.get("confidence", 0.0)),
            metadata=dict(data.get("metadata", {}) or {}),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ObstacleSpec:
    """Terrain obstacle primitive placeholder. All dimensions use meters."""

    obstacle_id: str
    kind: str
    pose: Pose3D
    dimensions_xyz_m: tuple[float, float, float]
    radius_m: float = 0.0
    collision_uri: str | None = None
    visual_uri: str | None = None
    material_id: str = ""
    is_fixed: bool = True
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.obstacle_id:
            raise ValueError("obstacle_id is required")
        if self.kind not in {"rock", "step", "gap", "ridge", "generic"}:
            raise ValueError("kind must be rock, step, gap, ridge, or generic")
        for index, value in enumerate(self.dimensions_xyz_m):
            _require_nonnegative(f"dimensions_xyz_m[{index}]", value)
        _require_nonnegative("radius_m", self.radius_m)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "ObstacleSpec":
        if "pose" in data:
            pose = Pose3D.from_mapping(data["pose"])
            dimensions = _tuple3(data.get("dimensions_xyz_m"), (0.0, 0.0, 0.0))
        else:
            warnings.warn(
                "ObstacleSpec x_m/y_m/height_m/width_m/length_m fields are deprecated; migrated to pose/dimensions.",
                DeprecationWarning,
                stacklevel=2,
            )
            pose = Pose3D((float(data.get("x_m", 0.0)), float(data.get("y_m", 0.0)), 0.0), (0.0, 0.0, 0.0))
            dimensions = (
                float(data.get("length_m", 0.0)),
                float(data.get("width_m", 0.0)),
                float(data.get("height_m", 0.0)),
            )
        return cls(
            obstacle_id=str(data["obstacle_id"]),
            kind=str(data.get("kind", "generic")),
            pose=pose,
            dimensions_xyz_m=dimensions,
            radius_m=float(data.get("radius_m", 0.0)),
            collision_uri=data.get("collision_uri"),
            visual_uri=data.get("visual_uri"),
            material_id=str(data.get("material_id", "") or ""),
            is_fixed=bool(data.get("is_fixed", True)),
            metadata=dict(data.get("metadata", {}) or {}),
        )

    @property
    def height_m(self) -> float:
        return self.dimensions_xyz_m[2]

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["pose"] = self.pose.to_dict()
        result["dimensions_xyz_m"] = list(self.dimensions_xyz_m)
        return result


@dataclass(frozen=True)
class TerrainScenario:
    """Environment-only terrain scenario contract. Scout response is not a primary field."""

    terrain_id: str
    display_name: str
    terrain_type: str
    surface_hint: str
    geometry: TerrainGeometrySpec
    material_id: str
    dimensions_xyz_m: tuple[float, float, float]
    frame_id: str
    random_seed: int
    obstacles: list[ObstacleSpec] = field(default_factory=list)
    slope_long_deg: float = 0.0
    slope_lat_deg: float = 0.0
    roughness_m: float = 0.0
    gap_width_m: float = 0.0
    patch_id: int = 1
    grid_x: int = 0
    grid_y: int = 0
    observation_state: str = "GEOMETRY_ONLY"
    geometry_confidence: float = 0.5
    prediction_confidence: float = 0.0
    legacy_scout_response: dict[str, float] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.terrain_id:
            raise ValueError("terrain_id is required")
        if self.terrain_type not in {"rigid", "rocky", "granular", "mixed", "unknown"}:
            raise ValueError("terrain_type must be rigid, rocky, granular, mixed, or unknown")
        if not self.material_id:
            raise ValueError("material_id is required")
        if not self.frame_id:
            raise ValueError("frame_id is required")
        for index, value in enumerate(self.dimensions_xyz_m):
            _require_positive(f"dimensions_xyz_m[{index}]", value)
        _require_nonnegative("roughness_m", self.roughness_m)
        _require_nonnegative("gap_width_m", self.gap_width_m)
        _require_confidence("geometry_confidence", self.geometry_confidence)
        _require_confidence("prediction_confidence", self.prediction_confidence)
        if self.observation_state not in OBSERVATION_STATES:
            raise ValueError("observation_state is invalid")
        if abs(self.slope_long_deg) > 89 or abs(self.slope_lat_deg) > 89:
            raise ValueError("slope angles must be within +/-89 degrees")

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "TerrainScenario":
        obstacles = [ObstacleSpec.from_mapping(item) for item in data.get("obstacles", []) or []]
        legacy_scout_response = {key: float(value) for key, value in dict(data.get("scout_response", {}) or {}).items()}
        if legacy_scout_response:
            warnings.warn(
                "TerrainScenario.scout_response is deprecated; load ScoutObservation from observations/ instead.",
                DeprecationWarning,
                stacklevel=2,
            )
        return cls(
            terrain_id=str(data["terrain_id"]),
            display_name=str(data.get("display_name") or data["terrain_id"]),
            terrain_type=str(data.get("terrain_type", "unknown")),
            surface_hint=str(data.get("surface_hint", "") or ""),
            geometry=TerrainGeometrySpec.from_mapping(data.get("geometry")),
            material_id=str(data.get("material_id", "terrain_material_unknown")),
            dimensions_xyz_m=_tuple3(data.get("dimensions_xyz_m"), (2.0, 2.0, 0.2)),
            frame_id=str(data.get("frame_id", data.get("geometry", {}).get("frame_id", "terrain"))),
            random_seed=int(data.get("random_seed", 0)),
            obstacles=obstacles,
            slope_long_deg=float(data.get("slope_long_deg", 0.0)),
            slope_lat_deg=float(data.get("slope_lat_deg", 0.0)),
            roughness_m=float(data.get("roughness_m", 0.0)),
            gap_width_m=float(data.get("gap_width_m", 0.0)),
            patch_id=int(data.get("patch_id", 1)),
            grid_x=int(data.get("grid_x", 0)),
            grid_y=int(data.get("grid_y", 0)),
            observation_state=str(data.get("observation_state", "GEOMETRY_ONLY")),
            geometry_confidence=float(data.get("geometry_confidence", 0.5)),
            prediction_confidence=float(data.get("prediction_confidence", 0.0)),
            legacy_scout_response=legacy_scout_response,
            metadata=dict(data.get("metadata", {}) or {}),
        )

    @property
    def max_obstacle_height_m(self) -> float:
        if not self.obstacles:
            return 0.0
        return max(obstacle.height_m for obstacle in self.obstacles)

    def legacy_observation(self, rover_id: str, control_profile_id: str) -> "ScoutObservation | None":
        if not self.legacy_scout_response:
            return None
        warnings.warn(
            "Using legacy TerrainScenario.scout_response as a temporary ScoutObservation.",
            DeprecationWarning,
            stacklevel=2,
        )
        payload = self.legacy_scout_response
        return ScoutObservation(
            observation_id=f"legacy_{self.terrain_id}",
            terrain_id=self.terrain_id,
            scout_rover_id=rover_id,
            control_profile_id=control_profile_id,
            timestamp_utc=datetime.now(timezone.utc).isoformat(),
            patch_id=self.patch_id,
            grid_x=self.grid_x,
            grid_y=self.grid_y,
            pose_xyz_m=(0.0, 0.0, 0.0),
            heading_deg=0.0,
            sample_count=1,
            travel_distance_m=0.0,
            slope_long_deg=self.slope_long_deg,
            slope_lat_deg=self.slope_lat_deg,
            roughness_m=self.roughness_m,
            obstacle_height_m=self.max_obstacle_height_m,
            gap_width_m=self.gap_width_m,
            mean_body_speed_mps=0.0,
            mean_wheel_speed_radps=0.0,
            mean_slip=float(payload["scout_slip"]),
            max_slip=float(payload.get("max_slip", payload["scout_slip"])),
            mean_sinkage_m=float(payload["scout_sinkage_m"]),
            max_sinkage_m=float(payload.get("max_sinkage_m", payload["scout_sinkage_m"])),
            mean_wheel_torque_nm=float(payload["scout_wheel_torque_nm"]),
            cot=float(payload["scout_cot"]),
            vibration_rms_g=float(payload["vibration_rms_g"]),
            geometry_confidence=self.geometry_confidence,
            response_confidence=0.35,
            source_type="manual",
            observation_state="SCOUT_OBSERVED",
            metadata={"migrated_from": "TerrainScenario.scout_response"},
        )

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["geometry"] = self.geometry.to_dict()
        result["dimensions_xyz_m"] = list(self.dimensions_xyz_m)
        result["obstacles"] = [obstacle.to_dict() for obstacle in self.obstacles]
        return result


@dataclass(frozen=True)
class ScoutObservation:
    observation_id: str
    terrain_id: str
    scout_rover_id: str
    control_profile_id: str
    timestamp_utc: str
    patch_id: int
    grid_x: int
    grid_y: int
    pose_xyz_m: tuple[float, float, float]
    heading_deg: float
    sample_count: int
    travel_distance_m: float
    slope_long_deg: float
    slope_lat_deg: float
    roughness_m: float
    obstacle_height_m: float
    gap_width_m: float
    mean_body_speed_mps: float
    mean_wheel_speed_radps: float
    mean_slip: float
    max_slip: float
    mean_sinkage_m: float
    max_sinkage_m: float
    mean_wheel_torque_nm: float
    cot: float
    vibration_rms_g: float
    geometry_confidence: float
    response_confidence: float
    source_type: str
    observation_state: str = "SCOUT_OBSERVED"
    prediction_confidence: float = 0.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.observation_id:
            raise ValueError("observation_id is required")
        if not self.terrain_id:
            raise ValueError("terrain_id is required")
        if not self.scout_rover_id:
            raise ValueError("scout_rover_id is required")
        if not self.control_profile_id:
            raise ValueError("control_profile_id is required")
        if self.sample_count <= 0:
            raise ValueError("sample_count must be positive")
        if self.source_type not in SOURCE_TYPES:
            raise ValueError("source_type must be mock, simulation, sensor_log, or manual")
        if self.observation_state not in OBSERVATION_STATES:
            raise ValueError("observation_state is invalid")
        for name in [
            "travel_distance_m",
            "roughness_m",
            "obstacle_height_m",
            "gap_width_m",
            "mean_body_speed_mps",
            "mean_wheel_speed_radps",
            "mean_slip",
            "max_slip",
            "mean_sinkage_m",
            "max_sinkage_m",
            "mean_wheel_torque_nm",
            "cot",
            "vibration_rms_g",
        ]:
            _require_nonnegative(name, float(getattr(self, name)))
        for name in ["mean_slip", "max_slip", "geometry_confidence", "response_confidence", "prediction_confidence"]:
            _require_confidence(name, float(getattr(self, name)))

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "ScoutObservation":
        return cls(
            observation_id=str(data["observation_id"]),
            terrain_id=str(data["terrain_id"]),
            scout_rover_id=str(data["scout_rover_id"]),
            control_profile_id=str(data["control_profile_id"]),
            timestamp_utc=str(data["timestamp_utc"]),
            patch_id=int(data["patch_id"]),
            grid_x=int(data["grid_x"]),
            grid_y=int(data["grid_y"]),
            pose_xyz_m=_tuple3(data.get("pose_xyz_m"), (0.0, 0.0, 0.0)),
            heading_deg=float(data.get("heading_deg", 0.0)),
            sample_count=int(data["sample_count"]),
            travel_distance_m=float(data["travel_distance_m"]),
            slope_long_deg=float(data["slope_long_deg"]),
            slope_lat_deg=float(data["slope_lat_deg"]),
            roughness_m=float(data["roughness_m"]),
            obstacle_height_m=float(data["obstacle_height_m"]),
            gap_width_m=float(data["gap_width_m"]),
            mean_body_speed_mps=float(data["mean_body_speed_mps"]),
            mean_wheel_speed_radps=float(data["mean_wheel_speed_radps"]),
            mean_slip=float(data["mean_slip"]),
            max_slip=float(data["max_slip"]),
            mean_sinkage_m=float(data["mean_sinkage_m"]),
            max_sinkage_m=float(data["max_sinkage_m"]),
            mean_wheel_torque_nm=float(data["mean_wheel_torque_nm"]),
            cot=float(data["cot"]),
            vibration_rms_g=float(data["vibration_rms_g"]),
            geometry_confidence=float(data["geometry_confidence"]),
            response_confidence=float(data["response_confidence"]),
            source_type=str(data["source_type"]),
            observation_state=str(data.get("observation_state", "SCOUT_OBSERVED")),
            prediction_confidence=float(data.get("prediction_confidence", 0.0)),
            metadata=dict(data.get("metadata", {}) or {}),
        )

    def to_scout_measurement(self, surface_hint: str = "") -> ScoutMeasurement:
        return ScoutMeasurement(
            patch_id=self.patch_id,
            grid_x=self.grid_x,
            grid_y=self.grid_y,
            slope_long_deg=self.slope_long_deg,
            slope_lat_deg=self.slope_lat_deg,
            roughness_m=self.roughness_m,
            obstacle_height_m=self.obstacle_height_m,
            gap_width_m=self.gap_width_m,
            scout_slip=self.mean_slip,
            scout_sinkage_m=self.mean_sinkage_m,
            scout_wheel_torque_nm=self.mean_wheel_torque_nm,
            scout_cot=self.cot,
            vibration_rms_g=self.vibration_rms_g,
            surface_hint=surface_hint,
        )

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["pose_xyz_m"] = list(self.pose_xyz_m)
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
class MobilityMetrics:
    completed: bool | None = None
    simulation_time_s: float | None = None
    travel_distance_m: float | None = None
    mean_body_speed_mps: float | None = None
    mean_slip: float | None = None
    p95_slip: float | None = None
    max_slip: float | None = None
    mean_sinkage_m: float | None = None
    max_sinkage_m: float | None = None
    peak_wheel_torque_nm: float | None = None
    mechanical_energy_j: float | None = None
    max_roll_deg: float | None = None
    max_pitch_deg: float | None = None
    min_tipover_margin_deg: float | None = None
    chassis_collision: bool | None = None
    wheel_stall: bool | None = None
    rollover: bool | None = None
    timeout: bool | None = None

    @classmethod
    def from_mapping(cls, data: dict[str, Any] | None) -> "MobilityMetrics":
        return cls(**(data or {}))

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
    metrics: dict[str, Any]
    risk_components: dict[str, float]
    final_risk: float | None
    grade: str
    metrics_typed: MobilityMetrics
    prediction_confidence: float
    model_status: str
    evaluation_state: str
    failure_reasons: list[str] = field(default_factory=list)
    hard_failure_reasons: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    artifacts: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.status not in {"ok", "mock", "not_implemented", "not_evaluated", "failed"}:
            raise ValueError("status must be ok, mock, not_implemented, not_evaluated, or failed")
        _require_nonnegative("duration_s", self.duration_s)
        if self.final_risk is not None and not 0.0 <= self.final_risk <= 1.0:
            raise ValueError("final_risk must be in 0..1 or None")
        if self.grade not in {"Safe", "Caution", "Risk", "Unknown", "NOT_EVALUATED"}:
            raise ValueError("grade must be Safe, Caution, Risk, Unknown, or NOT_EVALUATED")
        _require_confidence("prediction_confidence", self.prediction_confidence)
        if self.model_status not in MODEL_STATUSES:
            raise ValueError("model_status is invalid")
        if self.evaluation_state not in EVALUATION_STATES:
            raise ValueError("evaluation_state is invalid")
        if self.model_status == "mock" and self.evaluation_state != "NOT_EVALUATED":
            raise ValueError("mock result cannot be stored as evaluated")
        if self.evaluation_state == "NOT_EVALUATED" and self.final_risk is not None:
            raise ValueError("NOT_EVALUATED result must not have final_risk")

    @classmethod
    def new(
        cls,
        backend_name: str,
        rover_id: str,
        terrain_id: str,
        control_profile_id: str,
        status: str,
        duration_s: float,
        metrics: dict[str, Any],
        risk_components: dict[str, float],
        final_risk: float | None,
        grade: str,
        metrics_typed: MobilityMetrics | None = None,
        prediction_confidence: float = 0.0,
        model_status: str = "heuristic",
        evaluation_state: str = "EVALUATED",
        failure_reasons: list[str] | None = None,
        hard_failure_reasons: list[str] | None = None,
        notes: list[str] | None = None,
        artifacts: dict[str, Any] | None = None,
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
            metrics_typed=metrics_typed or MobilityMetrics(),
            prediction_confidence=prediction_confidence,
            model_status=model_status,
            evaluation_state=evaluation_state,
            failure_reasons=failure_reasons or [],
            hard_failure_reasons=hard_failure_reasons or [],
            notes=notes or [],
            artifacts=artifacts or {},
        )

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "SimulationResult":
        return cls(
            experiment_id=str(data["experiment_id"]),
            backend_name=str(data["backend_name"]),
            rover_id=str(data["rover_id"]),
            terrain_id=str(data["terrain_id"]),
            control_profile_id=str(data["control_profile_id"]),
            status=str(data["status"]),
            started_at_utc=str(data["started_at_utc"]),
            duration_s=float(data["duration_s"]),
            metrics=dict(data.get("metrics", {}) or {}),
            risk_components={key: float(value) for key, value in dict(data.get("risk_components", {}) or {}).items()},
            final_risk=None if data.get("final_risk") is None else float(data["final_risk"]),
            grade=str(data["grade"]),
            metrics_typed=MobilityMetrics.from_mapping(data.get("metrics_typed")),
            prediction_confidence=float(data.get("prediction_confidence", 0.0)),
            model_status=str(data.get("model_status", "heuristic")),
            evaluation_state=str(data.get("evaluation_state", "EVALUATED")),
            failure_reasons=list(data.get("failure_reasons", []) or []),
            hard_failure_reasons=list(data.get("hard_failure_reasons", []) or []),
            notes=list(data.get("notes", []) or []),
            artifacts=dict(data.get("artifacts", {}) or {}),
        )

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["metrics_typed"] = self.metrics_typed.to_dict()
        return result

