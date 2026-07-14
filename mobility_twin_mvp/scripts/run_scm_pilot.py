"""CLI entry point for the scout-to-main SCM transfer pilot.

Usage:
    conda activate chrono
    cd mobility_twin_mvp
    python scripts/run_scm_pilot.py --slope flat --soil loose

Requires pychrono.vehicle (see docs/ENVIRONMENT_SETUP.md). As of 2026-07-14
pychrono.vehicle fails to import in this project's `chrono` conda env (DLL
init error), so this script exits with a clear error rather than running --
it is a ready-to-go skeleton, not a verified pipeline. See
docs/SCOUT_MAIN_SCM_PILOT_PLAN.md.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import uuid
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.chrono.availability import get_pychrono_availability
from src.experiments.scm_pilot.evaluator import evaluate
from src.experiments.scm_pilot.metrics import TRAJECTORY_COLUMNS
from src.experiments.scm_pilot.predictor import identity_baseline, slope_and_soil, slope_only, user_formula
from src.experiments.scm_pilot.presets import DEFAULT_COMMAND, SLOPE_PRESETS_DEG, SOIL_PRESET_MATERIAL_IDS
from src.experiments.scm_pilot.runner import run_pilot
from src.experiments.scm_pilot.scenario import load_rover_spec, load_soil_material


def write_trajectory_csv(trajectory: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=TRAJECTORY_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(trajectory)


def main() -> int:
    parser = argparse.ArgumentParser(description="Scout-to-main SCM transfer pilot")
    parser.add_argument("--slope", choices=sorted(SLOPE_PRESETS_DEG), default="flat")
    parser.add_argument("--soil", choices=sorted(SOIL_PRESET_MATERIAL_IDS), default="loose")
    parser.add_argument("--out", type=Path, default=None, help="Output dir (default: data/scm_pilot/<run_id>)")
    args = parser.parse_args()

    availability = get_pychrono_availability()
    if not availability.pychrono_available:
        print(f"pychrono is not available: {availability.diagnostic_message}", file=sys.stderr)
        return 1
    try:
        import pychrono.vehicle  # noqa: F401
    except ImportError as exc:
        print(
            f"pychrono.vehicle failed to import ({exc}); the SCM pilot cannot run. "
            "See docs/ENVIRONMENT_SETUP.md.",
            file=sys.stderr,
        )
        return 1

    slope_deg = SLOPE_PRESETS_DEG[args.slope]
    run_id = f"{args.slope}_{args.soil}_{uuid.uuid4().hex[:8]}"
    out_dir = args.out or (PROJECT_ROOT / "data" / "scm_pilot" / run_id)

    print(f"Running scout_v01 on {args.soil}/{args.slope} ...")
    scout_result = run_pilot("scout", slope_deg, args.soil, DEFAULT_COMMAND)
    print(f"Running main_v01 (ground truth) on {args.soil}/{args.slope} ...")
    main_result = run_pilot("main", slope_deg, args.soil, DEFAULT_COMMAND)

    scout_rover = load_rover_spec("scout")
    main_rover = load_rover_spec("main")
    soil_material = load_soil_material(args.soil)
    friction_angle = float(soil_material.scm_parameters.get("mohr_friction_angle_deg", 28.0))
    janosi_shear = float(soil_material.scm_parameters.get("janosi_shear_m", 0.02))

    predictions = [
        identity_baseline(scout_result.summary),
        slope_only(scout_result.summary, scout_rover, main_rover, slope_deg),
        slope_and_soil(scout_result.summary, scout_rover, main_rover, slope_deg, friction_angle, janosi_shear),
        user_formula(),
    ]
    pilot_evaluation = evaluate(main_result.summary, predictions)

    write_trajectory_csv(scout_result.trajectory, out_dir / "scout_trajectory.csv")
    write_trajectory_csv(main_result.trajectory, out_dir / "main_trajectory.csv")
    result_payload = {
        "run_id": run_id,
        "slope_preset": args.slope,
        "slope_deg": slope_deg,
        "soil_preset": args.soil,
        "soil_material_id": soil_material.material_id,
        "scout_summary": scout_result.summary.to_dict(),
        "main_summary": main_result.summary.to_dict(),
        "evaluation": pilot_evaluation.to_dict(),
    }
    (out_dir / "result.json").write_text(json.dumps(result_payload, indent=2), encoding="utf-8")

    print(json.dumps(result_payload["evaluation"], indent=2))
    print(f"\nWrote {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
