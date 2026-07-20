"""Runs a single (rover, condition) rigid_transfer_pilot scenario in its own process.

Internal helper for scripts/run_rigid_transfer_pilot.py's per-condition
subprocess isolation -- see that script's module docstring for why. Not meant
to be run manually, though nothing stops you.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.experiments.rigid_transfer_pilot.metrics import TRAJECTORY_COLUMNS
from src.experiments.rigid_transfer_pilot.presets import CONDITIONS, DEFAULT_COMMAND, EXTRA_CONDITIONS
from src.experiments.rigid_transfer_pilot.runner import run_pilot


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rover", required=True, choices=["scout", "main"])
    parser.add_argument("--condition", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    condition = next((c for c in CONDITIONS + EXTRA_CONDITIONS if c.condition_id == args.condition), None)
    if condition is None:
        print(f"unknown condition_id {args.condition!r}", file=sys.stderr)
        return 1

    result = run_pilot(args.rover, condition, DEFAULT_COMMAND)

    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out / "trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=TRAJECTORY_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(result.trajectory)
    (args.out / "summary.json").write_text(json.dumps(result.summary.to_dict(), indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
