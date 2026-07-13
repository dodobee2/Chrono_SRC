from __future__ import annotations

from pathlib import Path

import pandas as pd

from .schemas import REQUIRED_MEASUREMENT_COLUMNS


def generate_sample_patches() -> pd.DataFrame:
    rows = [
        [1, 0, 0, 1, 1, 0.006, 0.005, 0.00, 0.03, 0.004, 0.4, 0.35, 0.05, "rigid_flat"],
        [2, 1, 0, 2, 0, 0.010, 0.008, 0.00, 0.04, 0.005, 0.5, 0.38, 0.06, ""],
        [3, 2, 0, 6, 2, 0.015, 0.012, 0.02, 0.06, 0.008, 0.7, 0.50, 0.08, "rigid_flat"],
        [4, 3, 0, 12, 3, 0.020, 0.020, 0.02, 0.07, 0.010, 0.9, 0.65, 0.10, "rigid_slope"],
        [5, 4, 0, 20, 4, 0.025, 0.030, 0.04, 0.10, 0.012, 1.2, 0.85, 0.12, "rigid_slope"],
        [6, 0, 1, 4, 2, 0.055, 0.045, 0.02, 0.12, 0.012, 1.1, 0.95, 0.42, "rocky_rough"],
        [7, 1, 1, 8, 5, 0.075, 0.060, 0.03, 0.16, 0.014, 1.4, 1.15, 0.65, ""],
        [8, 2, 1, 5, 4, 0.090, 0.070, 0.04, 0.18, 0.016, 1.6, 1.25, 0.85, "rocky_rough"],
        [9, 3, 1, 3, 2, 0.012, 0.012, 0.01, 0.32, 0.055, 1.0, 1.25, 0.18, "granular"],
        [10, 4, 1, 2, 1, 0.018, 0.018, 0.01, 0.22, 0.030, 0.8, 0.90, 0.15, "granular"],
        [11, 0, 2, 11, 7, 0.020, 0.020, 0.03, 0.36, 0.065, 1.3, 1.45, 0.22, "granular_slope"],
        [12, 1, 2, 16, 9, 0.024, 0.030, 0.05, 0.42, 0.080, 1.7, 1.75, 0.30, ""],
        [13, 2, 2, 5, 2, 0.025, 0.055, 0.03, 0.10, 0.012, 1.0, 0.85, 0.20, "obstacle_step"],
        [14, 3, 2, 7, 3, 0.030, 0.095, 0.05, 0.18, 0.020, 1.5, 1.30, 0.35, "obstacle_step"],
        [15, 4, 2, 8, 3, 0.040, 0.125, 0.06, 0.22, 0.028, 2.1, 1.65, 0.48, "obstacle_step"],
        [16, 0, 3, 4, 2, 0.018, 0.020, 0.18, 0.08, 0.009, 0.8, 0.70, 0.10, ""],
        [17, 1, 3, 6, 3, 0.020, 0.025, 0.36, 0.12, 0.012, 1.0, 0.95, 0.12, "obstacle_step"],
        [18, 2, 3, 18, 15, 0.070, 0.080, 0.16, 0.38, 0.060, 2.2, 2.10, 0.90, "mixed"],
        [19, 3, 3, 25, 18, 0.085, 0.110, 0.22, 0.50, 0.085, 2.8, 2.80, 1.10, ""],
        [20, 4, 3, 10, 22, 0.060, 0.100, 0.28, 0.45, 0.070, 2.5, 2.40, 0.95, "mixed"],
    ]
    return pd.DataFrame(rows, columns=REQUIRED_MEASUREMENT_COLUMNS)


def save_sample_csv(path: str | Path) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    generate_sample_patches().to_csv(output_path, index=False)
    return output_path


if __name__ == "__main__":
    save_sample_csv(Path(__file__).resolve().parents[1] / "data" / "sample_patches.csv")

