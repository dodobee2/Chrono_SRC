"""CLI entry point for the rigid-terrain Scout-to-Main transfer pilot.

Usage:
    conda activate chrono
    cd mobility_twin_mvp
    python scripts/run_rigid_transfer_pilot.py

Unlike src/experiments/scm_pilot, this only needs pychrono core (ChSystemNSC,
rigid terrain) -- it never imports pychrono.vehicle, so it runs today even
though the SCM pilot is blocked (see docs/ENVIRONMENT_SETUP.md).

Each (rover, condition) pair runs in its own subprocess with a timeout and
automatic retry (see _run_condition_isolated below), not in this process
directly. This project's Chrono scenario building has been observed to hang
unpredictably for 60s+ roughly 1-in-3-to-4 attempts, even for pure
pychrono-core work with no pychrono.vehicle involved (see
docs/ENVIRONMENT_SETUP.md's "Native loading is unpredictably slow" section).
Running 18 scenario builds (9 conditions x 2 rovers) back to back in a single
process made hitting at least one hang a near-certainty; isolating each one
means a hang only costs that one subprocess, not the whole run.
"""

from __future__ import annotations

import csv
import json
import subprocess
import sys
import uuid
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.chrono.availability import get_pychrono_availability
from src.experiments.rigid_transfer_pilot.evaluator import evaluate
from src.experiments.rigid_transfer_pilot.metrics import RunSummary
from src.experiments.rigid_transfer_pilot.predictor import PREDICTORS, PredictionResult, predict_main_from_scout
from src.experiments.rigid_transfer_pilot.presets import CONDITIONS, DEFAULT_COMMAND
from src.experiments.rigid_transfer_pilot.scenario import load_rover_spec, terrain_context_for

SINGLE_CONDITION_SCRIPT = Path(__file__).resolve().parent / "_run_single_condition.py"
PER_ATTEMPT_TIMEOUT_S = 60.0
MAX_ATTEMPTS = 4


class ConditionRunFailed(RuntimeError):
    pass


def run_condition_isolated(rover_key: str, condition_id: str, out_dir: Path) -> tuple[list[dict], RunSummary]:
    """Runs one (rover, condition) pair in its own subprocess, retrying on timeout.

    Returns (trajectory_rows, RunSummary). Raises ConditionRunFailed if every
    attempt times out or errors -- callers should let this abort the whole
    run rather than silently substituting a fake/empty result.
    """
    last_error = "unknown error"
    for attempt in range(1, MAX_ATTEMPTS + 1):
        print(f"  ({rover_key}/{condition_id}) attempt {attempt}/{MAX_ATTEMPTS} ...", flush=True)
        try:
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SINGLE_CONDITION_SCRIPT),
                    "--rover",
                    rover_key,
                    "--condition",
                    condition_id,
                    "--out",
                    str(out_dir),
                ],
                capture_output=True,
                text=True,
                timeout=PER_ATTEMPT_TIMEOUT_S,
            )
        except subprocess.TimeoutExpired:
            last_error = f"timed out after {PER_ATTEMPT_TIMEOUT_S:.0f}s"
            print(f"    {last_error}, retrying...", flush=True)
            continue
        if completed.returncode != 0:
            last_error = (completed.stderr or completed.stdout or "non-zero exit").strip().splitlines()[-1]
            print(f"    failed ({last_error}), retrying...", flush=True)
            continue

        summary_path = out_dir / "summary.json"
        trajectory_path = out_dir / "trajectory.csv"
        if not summary_path.exists() or not trajectory_path.exists():
            last_error = "subprocess exited 0 but did not write summary.json/trajectory.csv"
            print(f"    {last_error}, retrying...", flush=True)
            continue

        summary = RunSummary(**json.loads(summary_path.read_text(encoding="utf-8")))
        with trajectory_path.open(encoding="utf-8") as handle:
            trajectory = list(csv.DictReader(handle))
        print(f"    ok (distance={summary.distance_m:.3f}m, contact={summary.max_contact_count})", flush=True)
        return trajectory, summary

    raise ConditionRunFailed(f"({rover_key}/{condition_id}) did not complete after {MAX_ATTEMPTS} attempts: {last_error}")


def write_trajectory_csv(trajectory: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(trajectory[0]) if trajectory else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(trajectory)


def write_summary_csv(summaries: dict[str, RunSummary], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [summaries[condition.condition_id].to_dict() for condition in CONDITIONS]
    fieldnames = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_rows_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = sorted({key for row in rows for key in row}) if rows else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    availability = get_pychrono_availability()
    if not availability.pychrono_available:
        print(f"pychrono is not available: {availability.diagnostic_message}", file=sys.stderr)
        return 1

    run_id = f"rigid_pilot_{uuid.uuid4().hex[:8]}"
    out_dir = PROJECT_ROOT / "data" / "rigid_transfer_pilot" / run_id

    scout_spec = load_rover_spec("scout")
    main_spec = load_rover_spec("main")

    scout_summaries: dict[str, RunSummary] = {}
    main_summaries: dict[str, RunSummary] = {}
    predictions: dict[str, dict[str, PredictionResult]] = {}
    prediction_rows: list[dict] = []

    for condition in CONDITIONS:
        print(f"[{condition.condition_id}] scout ...")
        scout_trajectory, scout_summary = run_condition_isolated(
            "scout", condition.condition_id, out_dir / condition.condition_id / "scout"
        )
        print(f"[{condition.condition_id}] main (ground truth) ...")
        main_trajectory, main_summary = run_condition_isolated(
            "main", condition.condition_id, out_dir / condition.condition_id / "main"
        )
        scout_summaries[condition.condition_id] = scout_summary
        main_summaries[condition.condition_id] = main_summary

        write_trajectory_csv(scout_trajectory, out_dir / condition.condition_id / "scout_trajectory.csv")
        write_trajectory_csv(main_trajectory, out_dir / condition.condition_id / "main_trajectory.csv")

        terrain_context = terrain_context_for(condition)
        condition_predictions: dict[str, PredictionResult] = {}
        for predictor_name in PREDICTORS:
            prediction = predict_main_from_scout(
                predictor_name, scout_summary, scout_spec, main_spec, terrain_context, None
            )
            condition_predictions[predictor_name] = prediction
            row = {"condition_id": condition.condition_id, "predictor_name": predictor_name, "status": prediction.status}
            if prediction.metrics:
                row.update(prediction.metrics)
            prediction_rows.append(row)
        predictions[condition.condition_id] = condition_predictions

    condition_ids = [condition.condition_id for condition in CONDITIONS]
    pilot_evaluation = evaluate(condition_ids, main_summaries, predictions)

    write_summary_csv(scout_summaries, out_dir / "scout_results.csv")
    write_summary_csv(main_summaries, out_dir / "main_ground_truth.csv")
    write_rows_csv(prediction_rows, out_dir / "predictions.csv")
    write_rows_csv([c.to_dict() for c in pilot_evaluation.comparisons], out_dir / "comparison.csv")

    scenario_payload = {
        "run_id": run_id,
        "conditions": [
            {
                "condition_id": condition.condition_id,
                "slope_deg": condition.slope_deg,
                "friction_key": condition.friction_key,
                "obstacle_key": condition.obstacle_key,
            }
            for condition in CONDITIONS
        ],
        "command": {"torque_fraction": DEFAULT_COMMAND.torque_fraction, "duration_s": DEFAULT_COMMAND.duration_s},
        "rovers": {"scout": scout_spec.rover_id, "main": main_spec.rover_id},
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "scenario.json").write_text(json.dumps(scenario_payload, indent=2), encoding="utf-8")
    (out_dir / "summary.json").write_text(json.dumps(pilot_evaluation.to_dict(), indent=2), encoding="utf-8")

    print(json.dumps({"aggregates": pilot_evaluation.to_dict()["aggregates"]}, indent=2))
    print(f"\nWrote {out_dir}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ConditionRunFailed as exc:
        print(f"\nFATAL: {exc}", file=sys.stderr)
        sys.exit(1)
