"""Generic subprocess isolation with timeout + retry for any Chrono-touching script.

Chrono scenario building in this environment has been observed to hang
unpredictably for 60s+ roughly 1-in-3-to-4 attempts, even for pure
pychrono-core work (see docs/ENVIRONMENT_SETUP.md's "Native loading is
unpredictably slow" section). Never call pychrono in-process from a
long-lived caller (a Streamlit server, a test run) -- isolate each attempt in
its own subprocess with a hard timeout so a hang only costs that one attempt,
not the whole process.

src/experiments/rigid_transfer_pilot/isolated_runner.py predates this and has
its own copy specialized to that pilot's fixed --rover/--condition CLI
signature; this module is the generic version for scripts with arbitrary
argument lists (e.g. scripts/_run_live_rover_drive.py's --rover-id/
--terrain-id/--control-id), so a second caller doesn't need a third copy of
the same retry loop.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Callable

AttemptCallback = Callable[[int, int, str], None]


class IsolatedRunFailed(RuntimeError):
    pass


def run_script_isolated(
    script_path: Path,
    args: list[str],
    required_output_files: list[Path],
    timeout_s: float = 60.0,
    max_attempts: int = 4,
    on_attempt: AttemptCallback | None = None,
) -> subprocess.CompletedProcess:
    """Runs `python script_path *args` in its own subprocess, retrying on timeout/failure.

    Succeeds only when the subprocess exits 0 AND every path in
    required_output_files exists afterward -- a subprocess that exits 0
    without producing its expected output is treated as a failure, not a
    silent success. Raises IsolatedRunFailed if every attempt fails.

    on_attempt(attempt, max_attempts, status), if given, is called with
    status in {"starting", "timeout", "failed", "ok"} for progress reporting
    (e.g. a Streamlit spinner).
    """
    last_error = "unknown error"
    for attempt in range(1, max_attempts + 1):
        if on_attempt:
            on_attempt(attempt, max_attempts, "starting")
        try:
            completed = subprocess.run(
                [sys.executable, str(script_path), *args],
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
            stderr_tail = (completed.stderr or completed.stdout or "non-zero exit").strip()
            last_error = stderr_tail.splitlines()[-1] if stderr_tail else "non-zero exit"
            if on_attempt:
                on_attempt(attempt, max_attempts, "failed")
            continue
        missing = [str(p) for p in required_output_files if not p.exists()]
        if missing:
            last_error = f"subprocess exited 0 but missing output files: {missing}"
            if on_attempt:
                on_attempt(attempt, max_attempts, "failed")
            continue
        if on_attempt:
            on_attempt(attempt, max_attempts, "ok")
        return completed
    raise IsolatedRunFailed(f"did not complete after {max_attempts} attempts: {last_error}")
