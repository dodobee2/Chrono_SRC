"""검증 결과 CSV → PNG 플롯.

run_rover_check.py 가 자동 호출하며, 단독 실행도 가능:
    python scripts/plot_results.py                      # outputs/rover_check 전체
    python scripts/plot_results.py --csv path/to.csv    # 단일 CSV
(matplotlib 한글 폰트 문제로 라벨은 영어 사용 — 연습 레포 규칙)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from logger import read_csv  # noqa: E402

WHEELS = ("FL", "FR", "RL", "RR")
COLORS = {"FL": "#1f77b4", "FR": "#ff7f0e", "RL": "#2ca02c", "RR": "#d62728"}


def plot_run(csv_path: Path, png_path: Path | None = None,
             title: str | None = None) -> Path:
    """단일 시나리오 CSV → 2×2 패널 PNG."""
    data = read_csv(csv_path)
    t = data["t_s"]
    png_path = png_path or csv_path.with_suffix(".png")
    title = title or csv_path.stem

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle(title, fontsize=14)

    ax = axes[0][0]
    ax.plot(t, data["x_m"], label="x", color="#1f77b4")
    ax.plot(t, data["y_m"], label="y", color="#ff7f0e")
    ax.plot(t, data["z_m"], label="z", color="#2ca02c")
    ax.set_title("Chassis position")
    ax.set_ylabel("[m]")
    ax.legend(loc="best")

    ax = axes[0][1]
    ax.plot(t, data["v_forward_mps"], color="#1f77b4", label="v forward")
    ax2 = ax.twinx()
    ax2.plot(t, data["pitch_deg"], color="#9467bd", lw=1.0, label="pitch")
    ax2.plot(t, data["roll_deg"], color="#8c564b", lw=1.0, label="roll")
    ax2.set_ylabel("Attitude [deg]")
    ax.set_title("Forward speed / attitude")
    ax.set_ylabel("[m/s]")
    l1, la1 = ax.get_legend_handles_labels()
    l2, la2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, la1 + la2, loc="best")

    ax = axes[1][0]
    for w in WHEELS:
        ax.plot(t, data[f"slip_{w}"], color=COLORS[w], lw=1.0, label=w)
    ax.set_title("Wheel slip ratio")
    ax.set_ylabel("slip [-]")
    ax.set_ylim(-0.5, 0.5)
    ax.legend(loc="best", ncols=4)

    ax = axes[1][1]
    for w in WHEELS:
        ax.plot(t, [v * 1000 for v in data[f"torque_{w}_nm"]],
                color=COLORS[w], lw=1.0, label=w)
    ax2 = ax.twinx()
    ax2.plot(t, data["energy_j"], color="#7f7f7f", lw=1.4, ls="--",
             label="energy")
    ax2.set_ylabel("Energy proxy [J]")
    ax.set_title("Wheel torque / energy")
    ax.set_ylabel("torque [mN·m]")
    l1, la1 = ax.get_legend_handles_labels()
    l2, la2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, la1 + la2, loc="best", ncols=3)

    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("Time [s]")

    fig.savefig(png_path, dpi=160)
    plt.close(fig)
    return png_path


def plot_compare(csv_by_rover: dict[str, Path], scenario: str,
                 png_path: Path) -> Path:
    """여러 로버의 같은 시나리오 비교 PNG (속도·토크·에너지)."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    fig.suptitle(f"Rover comparison - {scenario}", fontsize=14)
    palette = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]

    for i, (rover_id, csv_path) in enumerate(csv_by_rover.items()):
        data = read_csv(csv_path)
        t = data["t_s"]
        c = palette[i % len(palette)]
        axes[0].plot(t, data["v_forward_mps"], color=c, label=rover_id)
        mean_tau = [
            sum(abs(data[f"torque_{w}_nm"][k]) for w in WHEELS) / 4 * 1000
            for k in range(len(t))
        ]
        axes[1].plot(t, mean_tau, color=c, label=rover_id)
        axes[2].plot(t, data["energy_j"], color=c, label=rover_id)

    for ax, (title, ylab) in zip(axes, [
        ("Forward speed", "[m/s]"),
        ("Mean |wheel torque|", "[mN·m]"),
        ("Energy proxy", "[J]"),
    ]):
        ax.set_title(title)
        ax.set_ylabel(ylab)
        ax.set_xlabel("Time [s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

    fig.savefig(png_path, dpi=160)
    plt.close(fig)
    return png_path


def plot_all(out_root: Path, rover_ids: list[str]) -> list[Path]:
    """rover_check 출력 트리 전체에 대해 PNG 생성."""
    made = []
    for rid in rover_ids:
        for stem in ("settle", "cruise_flat", "cruise_slope", "pivot_turn"):
            csv_path = out_root / rid / f"{stem}.csv"
            if csv_path.exists():
                made.append(plot_run(csv_path, title=f"{rid} - {stem}"))
    for stem in ("cruise_flat", "cruise_slope"):
        csvs = {rid: out_root / rid / f"{stem}.csv" for rid in rover_ids
                if (out_root / rid / f"{stem}.csv").exists()}
        if len(csvs) >= 2:
            made.append(plot_compare(csvs, stem, out_root / f"compare_{stem}.png"))
    for p in made:
        print(f"plot: {p}")
    return made


def main() -> None:
    ap = argparse.ArgumentParser(description="검증 결과 플롯")
    ap.add_argument("--csv", type=Path, help="단일 CSV 플롯")
    ap.add_argument("--out-root", type=Path,
                    default=PROJECT_ROOT / "outputs" / "rover_check")
    args = ap.parse_args()

    if args.csv:
        print(f"plot: {plot_run(args.csv)}")
        return
    rover_ids = sorted(
        p.name for p in args.out_root.iterdir() if p.is_dir()
    ) if args.out_root.exists() else []
    plot_all(args.out_root, rover_ids)


if __name__ == "__main__":
    main()
