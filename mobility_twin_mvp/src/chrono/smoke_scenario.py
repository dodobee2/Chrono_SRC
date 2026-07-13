from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import csv
import math
import time
import traceback
from typing import Any

from .availability import get_pychrono_availability


@dataclass(frozen=True)
class SmokeScenarioConfig:
    duration_s: float = 3.0
    step_size_s: float = 0.001
    sample_period_s: float = 0.02
    box_mass_kg: float = 1.0
    box_initial_position_m: tuple[float, float, float] = (0.0, 0.0, 1.0)
    gravity_mps2: tuple[float, float, float] = (0.0, 0.0, -9.81)

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["box_initial_position_m"] = list(self.box_initial_position_m)
        result["gravity_mps2"] = list(self.gravity_mps2)
        return result


@dataclass(frozen=True)
class SmokeScenarioResult:
    status: str
    metrics: dict[str, Any]
    trajectory: list[dict[str, float]]
    runner_log: str
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_smoke_scenario(config: SmokeScenarioConfig | None = None) -> SmokeScenarioResult:
    """Run a headless PyChrono box-drop smoke scenario.

    PyChrono is imported only inside this function so the rest of the app remains
    import-safe when Chrono is not installed.
    """
    config = config or SmokeScenarioConfig()
    availability = get_pychrono_availability()
    if not availability.pychrono_available:
        return SmokeScenarioResult(
            status="failed",
            metrics={"wall_time_s": 0.0, "contact_detected": False},
            trajectory=[],
            runner_log=availability.diagnostic_message,
            error=availability.diagnostic_message,
        )

    started = time.perf_counter()
    try:
        import pychrono as chrono  # type: ignore

        system = _make_system(chrono)
        _set_gravity(system, chrono, config.gravity_mps2)
        floor = _make_floor(system, chrono)
        box = _make_box(system, chrono, config)
        system.Add(floor)
        system.Add(box)

        trajectory = _simulate(system, box, chrono, config)
        wall_time = time.perf_counter() - started
        metrics = _extract_metrics(trajectory, config, wall_time)
        return SmokeScenarioResult(
            status="completed",
            metrics=metrics,
            trajectory=trajectory,
            runner_log="PyChrono smoke scenario completed.",
            error=None,
        )
    except Exception as exc:  # pragma: no cover - depends on local PyChrono API
        return SmokeScenarioResult(
            status="failed",
            metrics={"wall_time_s": time.perf_counter() - started, "contact_detected": False},
            trajectory=[],
            runner_log=traceback.format_exc(),
            error=f"{type(exc).__name__}: {exc}",
        )


def write_trajectory_csv(trajectory: list[dict[str, float]], output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        "time_s",
        "position_x_m",
        "position_y_m",
        "position_z_m",
        "velocity_x_mps",
        "velocity_y_mps",
        "velocity_z_mps",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(trajectory)
    return output_path


def validate_trajectory_schema(trajectory: list[dict[str, float]]) -> None:
    required = {
        "time_s",
        "position_x_m",
        "position_y_m",
        "position_z_m",
        "velocity_x_mps",
        "velocity_y_mps",
        "velocity_z_mps",
    }
    for row in trajectory:
        missing = required - set(row)
        if missing:
            raise ValueError(f"trajectory row missing columns: {sorted(missing)}")


def _make_system(chrono: Any) -> Any:  # pragma: no cover - requires PyChrono
    if hasattr(chrono, "ChSystemNSC"):
        return chrono.ChSystemNSC()
    if hasattr(chrono, "ChSystemSMC"):
        return chrono.ChSystemSMC()
    raise RuntimeError("No supported Chrono system class found: expected ChSystemNSC or ChSystemSMC")


def _set_gravity(system: Any, chrono: Any, gravity: tuple[float, float, float]) -> None:  # pragma: no cover
    vector = _vec(chrono, *gravity)
    if hasattr(system, "SetGravitationalAcceleration"):
        system.SetGravitationalAcceleration(vector)
    elif hasattr(system, "Set_G_acc"):
        system.Set_G_acc(vector)
    else:
        raise RuntimeError("No supported gravity setter found on Chrono system")


def _make_floor(system: Any, chrono: Any) -> Any:  # pragma: no cover
    floor = _make_body(chrono)
    _set_fixed(floor, True)
    _set_position(floor, chrono, 0.0, 0.0, -0.05)
    _set_mass(floor, 1.0)
    _add_box_collision(floor, chrono, 5.0, 5.0, 0.1)
    _enable_collision(floor, True)
    _set_visual_shape_box(floor, chrono, 5.0, 5.0, 0.1)
    return floor


def _make_box(system: Any, chrono: Any, config: SmokeScenarioConfig) -> Any:  # pragma: no cover
    box = _make_body(chrono)
    _set_mass(box, config.box_mass_kg)
    _set_position(box, chrono, *config.box_initial_position_m)
    _add_box_collision(box, chrono, 0.2, 0.2, 0.2)
    _enable_collision(box, True)
    _set_visual_shape_box(box, chrono, 0.2, 0.2, 0.2)
    return box


def _simulate(system: Any, box: Any, chrono: Any, config: SmokeScenarioConfig) -> list[dict[str, float]]:  # pragma: no cover
    trajectory: list[dict[str, float]] = []
    steps = int(config.duration_s / config.step_size_s)
    sample_every = max(1, int(config.sample_period_s / config.step_size_s))
    for step_index in range(steps + 1):
        if step_index % sample_every == 0 or step_index == steps:
            trajectory.append(_sample_state(system, box))
        if step_index < steps:
            system.DoStepDynamics(config.step_size_s)
    validate_trajectory_schema(trajectory)
    return trajectory


def _extract_metrics(
    trajectory: list[dict[str, float]],
    config: SmokeScenarioConfig,
    wall_time_s: float,
) -> dict[str, Any]:
    if not trajectory:
        return {"wall_time_s": wall_time_s, "contact_detected": False}
    first = trajectory[0]
    last = trajectory[-1]
    speeds = [
        math.sqrt(row["velocity_x_mps"] ** 2 + row["velocity_y_mps"] ** 2 + row["velocity_z_mps"] ** 2)
        for row in trajectory
    ]
    travel_distance = math.sqrt(
        (last["position_x_m"] - first["position_x_m"]) ** 2
        + (last["position_y_m"] - first["position_y_m"]) ** 2
        + (last["position_z_m"] - first["position_z_m"]) ** 2
    )
    contact_detected = any(row["position_z_m"] <= 0.16 and abs(row["velocity_z_mps"]) < 0.5 for row in trajectory)
    return {
        "simulation_time_s": config.duration_s,
        "travel_distance_m": travel_distance,
        "mean_body_speed_mps": travel_distance / max(config.duration_s, 1e-9),
        "final_position_xyz_m": [
            last["position_x_m"],
            last["position_y_m"],
            last["position_z_m"],
        ],
        "max_speed_mps": max(speeds),
        "contact_detected": contact_detected,
        "wall_time_s": wall_time_s,
    }


def _sample_state(system: Any, body: Any) -> dict[str, float]:  # pragma: no cover
    pos = body.GetPos()
    vel = body.GetPosDt() if hasattr(body, "GetPosDt") else body.GetLinVel()
    return {
        "time_s": float(system.GetChTime()),
        "position_x_m": float(pos.x),
        "position_y_m": float(pos.y),
        "position_z_m": float(pos.z),
        "velocity_x_mps": float(vel.x),
        "velocity_y_mps": float(vel.y),
        "velocity_z_mps": float(vel.z),
    }


def _make_body(chrono: Any) -> Any:  # pragma: no cover
    if hasattr(chrono, "ChBody"):
        return chrono.ChBody()
    raise RuntimeError("No supported ChBody class found")


def _vec(chrono: Any, x: float, y: float, z: float) -> Any:  # pragma: no cover
    if hasattr(chrono, "ChVector3d"):
        return chrono.ChVector3d(x, y, z)
    if hasattr(chrono, "ChVectorD"):
        return chrono.ChVectorD(x, y, z)
    if hasattr(chrono, "ChVector"):
        return chrono.ChVector(x, y, z)
    raise RuntimeError("No supported Chrono vector class found")


def _set_fixed(body: Any, fixed: bool) -> None:  # pragma: no cover
    if hasattr(body, "SetFixed"):
        body.SetFixed(fixed)
    elif hasattr(body, "SetBodyFixed"):
        body.SetBodyFixed(fixed)
    else:
        raise RuntimeError("No supported fixed-body setter found")


def _set_position(body: Any, chrono: Any, x: float, y: float, z: float) -> None:  # pragma: no cover
    body.SetPos(_vec(chrono, x, y, z))


def _set_mass(body: Any, mass: float) -> None:  # pragma: no cover
    body.SetMass(mass)


def _enable_collision(body: Any, enabled: bool) -> None:  # pragma: no cover
    if hasattr(body, "EnableCollision"):
        body.EnableCollision(enabled)
    elif hasattr(body, "SetCollide"):
        body.SetCollide(enabled)
    else:
        raise RuntimeError("No supported collision enable method found")


def _add_box_collision(body: Any, chrono: Any, x: float, y: float, z: float) -> None:  # pragma: no cover
    if hasattr(chrono, "ChCollisionShapeBox") and hasattr(body, "AddCollisionShape"):
        material = _make_contact_material(chrono)
        body.AddCollisionShape(chrono.ChCollisionShapeBox(material, x, y, z))
        return
    if hasattr(body, "GetCollisionModel"):
        model = body.GetCollisionModel()
        if hasattr(model, "ClearModel"):
            model.ClearModel()
        if hasattr(model, "AddBox"):
            try:
                model.AddBox(x / 2.0, y / 2.0, z / 2.0)
            except TypeError:
                model.AddBox(_make_contact_material(chrono), x / 2.0, y / 2.0, z / 2.0)
        if hasattr(model, "BuildModel"):
            model.BuildModel()
        return
    raise RuntimeError("No supported box collision API found")


def _make_contact_material(chrono: Any) -> Any:  # pragma: no cover
    if hasattr(chrono, "ChContactMaterialNSC"):
        return chrono.ChContactMaterialNSC()
    if hasattr(chrono, "ChMaterialSurfaceNSC"):
        return chrono.ChMaterialSurfaceNSC()
    if hasattr(chrono, "ChContactMaterialSMC"):
        return chrono.ChContactMaterialSMC()
    return None


def _set_visual_shape_box(body: Any, chrono: Any, x: float, y: float, z: float) -> None:  # pragma: no cover
    if hasattr(chrono, "ChVisualShapeBox") and hasattr(body, "AddVisualShape"):
        body.AddVisualShape(chrono.ChVisualShapeBox(x, y, z))

