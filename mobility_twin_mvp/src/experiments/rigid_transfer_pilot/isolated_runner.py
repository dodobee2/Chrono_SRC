"""Runs one (rover, condition) rigid_transfer_pilot scenario in an isolated subprocess.

Shared by scripts/run_rigid_transfer_pilot.py (CLI) and app.py's live-Chrono
Streamlit panel. Extracted here instead of living only in the CLI script so
both callers use one implementation.

This must run out-of-process, not in-process: building a Chrono scenario in
this environment has been observed to hang unpredictably for 60s+ roughly
1-in-3-to-4 attempts, even for pure pychrono-core work with no
pychrono.vehicle involved (see docs/ENVIRONMENT_SETUP.md's "Native loading is
unpredictably slow" section). A caller that is a long-lived process (a
Streamlit server, a test run) cannot afford to have that hang take it down;
isolating each attempt in its own subprocess with a hard timeout means a hang
only costs that one attempt, and it is simply retried.
"""

from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path
from typing import Callable

from .metrics import RunSummary

PROJECT_ROOT = Path(__file__).resolve().parents[3]
SINGLE_CONDITION_SCRIPT = PROJECT_ROOT / "scripts" / "_run_single_condition.py"
PER_ATTEMPT_TIMEOUT_S = 60.0
MAX_ATTEMPTS = 4

AttemptCallback = Callable[[int, int, str], None]


class ConditionRunFailed(RuntimeError):
    pass


def run_condition_isolated(
    rover_key: str,
    condition_id: str,
    out_dir: Path,
    timeout_s: float = PER_ATTEMPT_TIMEOUT_S,
    max_attempts: int = MAX_ATTEMPTS,
    on_attempt: AttemptCallback | None = None,
) -> tuple[list[dict], RunSummary]:
    """Runs one (rover, condition) pair in its own subprocess, retrying on timeout.

    Returns (trajectory_rows, RunSummary). Raises ConditionRunFailed if every
    attempt times out or errors -- callers should let this abort rather than
    silently substituting a fake/empty result.

    on_attempt(attempt, max_attempts, status), if given, is called with
    status in {"starting", "timeout", "failed", "ok"} so a caller (e.g. a
    Streamlit spinner) can show progress across retries.
    """
    last_error = "unknown error"
    for attempt in range(1, max_attempts + 1):
        if on_attempt:
            on_attempt(attempt, max_attempts, "starting")
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
                timeout=timeout_s,
            )
        except subprocess.TimeoutExpired:
            last_error = f"timed out after {timeout_s:.0f}s"
            if on_attempt:
                on_attempt(attempt, max_attempts, "timeout")
            continue
        if completed.returncode != 0:
            last_error = (completed.stderr or completed.stdout or "non-zero exit").strip().splitlines()[-1]
            if on_attempt:
                on_attempt(attempt, max_attempts, "failed")
            continue

        summary_path = out_dir / "summary.json"
        trajectory_path = out_dir / "trajectory.csv"
        if not summary_path.exists() or not trajectory_path.exists():
            last_error = "subprocess exited 0 but did not write summary.json/trajectory.csv"
            if on_attempt:
                on_attempt(attempt, max_attempts, "failed")
            continue

        summary = RunSummary(**json.loads(summary_path.read_text(encoding="utf-8")))
        with trajectory_path.open(encoding="utf-8") as handle:
            trajectory = list(csv.DictReader(handle))
        if on_attempt:
            on_attempt(attempt, max_attempts, "ok")
        return trajectory, summary

    raise ConditionRunFailed(f"({rover_key}/{condition_id}) did not complete after {max_attempts} attempts: {last_error}")
