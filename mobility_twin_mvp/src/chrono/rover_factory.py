"""Adapts Contract v2 RoverSpec into the vendored rover_module_v01 builder.

RoverSpec (src/integration_schemas.py) intentionally does not carry Chrono
execution-only fields (wheel_count, command_type, max_wheel_speed_radps): the
Contract is meant to stay usable by the heuristic backend without any
PyChrono knowledge. Those fields are read from RoverSpec.metadata instead --
see rover_models/scout_v01/rover.yaml and rover_models/main_v01/rover.yaml
for the convention.

Field mapping notes:
    - cg_height_m -> cg_xyz_m = (0, 0, cg_height_m). Contract v2 only tracks
      CG height; lateral/longitudinal CG offset is assumed zero until a real
      CAD/measured value is available.
    - wheel_material_id / wheel_contact_model / fallback_mu_eff / fallback_crr
      are heuristic-backend-only fields and are not used here.
"""

from __future__ import annotations

from dataclasses import fields as dataclass_fields
from typing import Any, TYPE_CHECKING

from ..integration_schemas import RoverSpec as ContractRoverSpec
from .vendor.rover_module_v01.rover_schema import RoverSpec as ChronoRoverSpec
from .vendor.rover_module_v01.rover_schema import SimConfig, validate_spec

if TYPE_CHECKING:
    from .vendor.rover_module_v01.rover_builder import RoverInstance

DEFAULT_WHEEL_COUNT = 4
DEFAULT_COMMAND_TYPE = "wheel_speed"


def _required_metadata_float(rover: ContractRoverSpec, key: str) -> float:
    value = rover.metadata.get(key)
    if value is None:
        raise ValueError(
            f"RoverSpec.metadata['{key}'] is required to build a PyChrono rover "
            f"(rover_id={rover.rover_id!r}); add it to rover.yaml metadata. "
            "This field is Chrono-execution-only and is not part of Contract v2."
        )
    return float(value)


def to_chrono_rover_spec(rover: ContractRoverSpec) -> ChronoRoverSpec:
    """Translate Contract v2 RoverSpec -> vendored rover_module_v01 RoverSpec."""
    command_type = str(rover.metadata.get("command_type", DEFAULT_COMMAND_TYPE))
    wheel_count = int(rover.metadata.get("wheel_count", DEFAULT_WHEEL_COUNT))
    max_wheel_speed_radps = _required_metadata_float(rover, "max_wheel_speed_radps")
    return ChronoRoverSpec(
        rover_id=rover.rover_id,
        mass_kg=rover.mass_kg,
        cg_xyz_m=(0.0, 0.0, rover.cg_height_m),
        wheel_count=wheel_count,
        driven_wheel_count=rover.driven_wheel_count,
        wheel_radius_m=rover.wheel_radius_m,
        wheel_width_m=rover.wheel_width_m,
        wheelbase_m=rover.wheelbase_m,
        track_width_m=rover.track_width_m,
        ground_clearance_m=rover.ground_clearance_m,
        command_type=command_type,
        max_wheel_torque_nm=rover.max_wheel_torque_nm,
        max_wheel_speed_radps=max_wheel_speed_radps,
    )


def to_sim_config(rover: ContractRoverSpec, overrides: dict[str, Any] | None = None) -> SimConfig:
    """Build a SimConfig from RoverSpec.metadata['sim'] plus explicit overrides."""
    payload = dict(rover.metadata.get("sim", {}) or {})
    if overrides:
        payload.update(overrides)
    valid_fields = {f.name for f in dataclass_fields(SimConfig)}
    unknown = set(payload) - valid_fields
    if unknown:
        raise ValueError(f"Unknown SimConfig fields in metadata['sim']: {sorted(unknown)}")
    if payload.get("chassis_size_m") is not None:
        payload["chassis_size_m"] = tuple(float(v) for v in payload["chassis_size_m"])
    return SimConfig(**payload)


def build_rover_from_spec(
    system: Any,
    rover: ContractRoverSpec,
    spawn_frame: Any = None,
    color: tuple[float, float, float] = (0.12, 0.38, 0.82),
    sim_overrides: dict[str, Any] | None = None,
) -> "RoverInstance":
    """Build a PyChrono rover body/motor set in `system` from a Contract v2 RoverSpec.

    Builder logic vendored from handoff/rover_module_v01 (Hojin, 2026-07-14).
    Calls apply_collision_defaults(cfg), which mutates global
    ChCollisionModel defaults -- call this before building any other bodies
    that must share the same collision envelope/margin.
    """
    from .vendor.rover_module_v01.rover_builder import apply_collision_defaults, build_rover

    chrono_spec = to_chrono_rover_spec(rover)
    validate_spec(chrono_spec)
    cfg = to_sim_config(rover, sim_overrides)
    apply_collision_defaults(cfg)
    return build_rover(system, chrono_spec, cfg, spawn_frame=spawn_frame, color=color)
