"""Extracts a RunSummary from a rigid-terrain pilot trajectory (no pychrono import needed)."""

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
    "contact_count",
]


@dataclass(frozen=True)
class RunSummary:
    rover_id: str
    condition_id: str
    distance_m: float
    mean_speed_mps: float
    mean_slip: float
    max_slip: float
    mean_wheel_torque_nm: float
    peak_wheel_torque_nm: float
    energy_j: float
    mean_pitch_deg: float
    mean_roll_deg: float
    final_pitch_deg: float
    final_roll_deg: float
    max_contact_count: int
    completed: bool
    wall_time_s: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def summarize(
    trajectory: list[dict[str, float]],
    rover_id: str,
    condition_id: str,
    wall_time_s: float,
    min_distance_for_completion_m: float = 0.5,
    max_pitch_or_roll_for_completion_deg: float = 45.0,
) -> RunSummary:
    if not trajectory:
        raise ValueError("cannot summarize an empty trajectory")
    first, last = trajectory[0], trajectory[-1]
    distance_m = ((last["x_m"] - first["x_m"]) ** 2 + (last["y_m"] - first["y_m"]) ** 2) ** 0.5
    duration_s = max(last["t_s"] - first["t_s"], 1e-9)
    slips = [row["mean_slip"] for row in trajectory]
    torques = [row["mean_wheel_torque_nm"] for row in trajectory]
    pitches = [row["pitch_deg"] for row in trajectory]
    rolls = [row["roll_deg"] for row in trajectory]
    max_contact_count = max((int(row["contact_count"]) for row in trajectory), default=0)

    completed = (
        distance_m >= min_distance_for_completion_m
        and abs(last["pitch_deg"]) < max_pitch_or_roll_for_completion_deg
        and abs(last["roll_deg"]) < max_pitch_or_roll_for_completion_deg
    )
    return RunSummary(
        rover_id=rover_id,
        condition_id=condition_id,
        distance_m=distance_m,
        mean_speed_mps=distance_m / duration_s,
        mean_slip=_mean(slips),
        max_slip=max((abs(v) for v in slips), default=0.0),
        mean_wheel_torque_nm=_mean(torques),
        peak_wheel_torque_nm=max((abs(v) for v in torques), default=0.0),
        energy_j=last["energy_j"],
        mean_pitch_deg=_mean([abs(v) for v in pitches]),
        mean_roll_deg=_mean([abs(v) for v in rolls]),
        final_pitch_deg=last["pitch_deg"],
        final_roll_deg=last["roll_deg"],
        max_contact_count=max_contact_count,
        completed=completed,
        wall_time_s=wall_time_s,
    )
