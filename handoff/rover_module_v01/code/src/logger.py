"""시뮬레이션 CSV 로거.

기록 항목: t, 위치(x,y,z), 자세(roll/pitch/yaw), 전진속도, 휠 각속도,
슬립률, 휠 토크 요구량, 총 출력, 에너지 proxy(∫Σ|τω| dt).
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

from rover_builder import WHEEL_NAMES, RoverInstance


class RoverLogger:
    def __init__(self, path: str | Path, rover: RoverInstance):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.rover = rover
        self._fh = self.path.open("w", newline="", encoding="utf-8")

        cols = ["t_s", "x_m", "y_m", "z_m",
                "roll_deg", "pitch_deg", "yaw_deg", "v_forward_mps"]
        cols += [f"omega_{n}_radps" for n in WHEEL_NAMES]
        cols += [f"slip_{n}" for n in WHEEL_NAMES]
        cols += [f"torque_{n}_nm" for n in WHEEL_NAMES]
        cols += ["power_w", "energy_j", "command_L", "command_R"]
        self._writer = csv.DictWriter(self._fh, fieldnames=cols)
        self._writer.writeheader()

    def log(self, t: float) -> None:
        r = self.rover
        pos = r.chassis.GetPos()
        roll, pitch, yaw = r.rpy_rad()
        omegas = r.wheel_omegas()
        slips = r.slip_ratios()
        torques = r.wheel_torques()

        row = {
            "t_s": f"{t:.4f}",
            "x_m": f"{pos.x:.6f}", "y_m": f"{pos.y:.6f}", "z_m": f"{pos.z:.6f}",
            "roll_deg": f"{math.degrees(roll):.4f}",
            "pitch_deg": f"{math.degrees(pitch):.4f}",
            "yaw_deg": f"{math.degrees(yaw):.4f}",
            "v_forward_mps": f"{r.forward_speed():.6f}",
            "power_w": f"{r.total_power_w():.6f}",
            "energy_j": f"{r.energy_proxy_j():.6f}",
        }
        cmd_l, cmd_r = r.commanded_lr()
        row["command_L"] = f"{cmd_l:.6f}"
        row["command_R"] = f"{cmd_r:.6f}"
        for n in WHEEL_NAMES:
            row[f"omega_{n}_radps"] = f"{omegas[n]:.6f}"
            row[f"slip_{n}"] = f"{slips[n]:.6f}"
            row[f"torque_{n}_nm"] = f"{torques.get(n, 0.0):.6f}"
        self._writer.writerow(row)

    def close(self) -> None:
        self._fh.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def read_csv(path: str | Path) -> dict[str, list[float]]:
    """CSV 를 컬럼별 float 리스트로 읽는다 (플롯/검증용)."""
    with Path(path).open("r", newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        data: dict[str, list[float]] = {k: [] for k in reader.fieldnames or []}
        for row in reader:
            for k, v in row.items():
                data[k].append(float(v))
    return data
