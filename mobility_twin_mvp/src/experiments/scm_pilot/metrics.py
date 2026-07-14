"""Extracts a RunSummary from a pilot trajectory (no pychrono import needed)."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

TRAJECTORY_COLUMNS = [
    "t_s",
    "x_m",
    "y_m",
    "z_m",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "v_forward_mps",
    "mean_slip",
    "mean_wheel_torque_nm",
    "power_w",
    "energy_j",
    "mean_sinkage_m",
    "max_sinkage_m",
]


@dataclass(frozen=True)
class RunSummary:
    rover_id: str
    slope_deg: float
    soil_material_id: str
    distance_m: float
    mean_speed_mps: float
    mean_slip: float
    max_slip: float
    mean_wheel_torque_nm: float
    peak_wheel_torque_nm: float
    energy_j: float
    mean_sinkage_m: float | None
    max_sinkage_m: float | None
    final_pitch_deg: float
    final_roll_deg: float
    completed: bool
    wall_time_s: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def summarize(
    trajectory: list[dict[str, float]],
    rover_id: str,
    slope_deg: float,
    soil_material_id: str,
    wall_time_s: float,
    min_distance_for_completion_m: float = 0.3,
    max_pitch_or_roll_for_completion_deg: float = 45.0,
) -> RunSummary:
    """Summarize a trajectory. mean/max_sinkage_m are None if the runner could
    not estimate sinkage (see runner.py::_wheel_sinkage_m).
    """
    if not trajectory:
        raise ValueError("cannot summarize an empty trajectory")
    first, last = trajectory[0], trajectory[-1]
    distance_m = ((last["x_m"] - first["x_m"]) ** 2 + (last["y_m"] - first["y_m"]) ** 2) ** 0.5
    duration_s = max(last["t_s"] - first["t_s"], 1e-9)
    slips = [row["mean_slip"] for row in trajectory]
    torques = [row["mean_wheel_torque_nm"] for row in trajectory]
    sinkages = [row["mean_sinkage_m"] for row in trajectory if row.get("mean_sinkage_m") is not None]
    max_sinkages = [row["max_sinkage_m"] for row in trajectory if row.get("max_sinkage_m") is not None]

    completed = (
        distance_m >= min_distance_for_completion_m
        and abs(last["pitch_deg"]) < max_pitch_or_roll_for_completion_deg
        and abs(last["roll_deg"]) < max_pitch_or_roll_for_completion_deg
    )
    return RunSummary(
        rover_id=rover_id,
        slope_deg=slope_deg,
        soil_material_id=soil_material_id,
        distance_m=distance_m,
        mean_speed_mps=distance_m / duration_s,
        mean_slip=_mean(slips),
        max_slip=max((abs(v) for v in slips), default=0.0),
        mean_wheel_torque_nm=_mean(torques),
        peak_wheel_torque_nm=max((abs(v) for v in torques), default=0.0),
        energy_j=last["energy_j"],
        mean_sinkage_m=_mean(sinkages) if sinkages else None,
        max_sinkage_m=max(max_sinkages) if max_sinkages else None,
        final_pitch_deg=last["pitch_deg"],
        final_roll_deg=last["roll_deg"],
        completed=completed,
        wall_time_s=wall_time_s,
    )
