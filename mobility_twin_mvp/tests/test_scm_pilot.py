from __future__ import annotations

import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.experiments.scm_pilot.evaluator import evaluate
from src.experiments.scm_pilot.metrics import RunSummary, summarize
from src.experiments.scm_pilot.predictor import identity_baseline, slope_and_soil, slope_only, user_formula
from src.registries import RoverRegistry


def rover_registry() -> RoverRegistry:
    return RoverRegistry(PROJECT_ROOT / "rover_models", repo_root=PROJECT_ROOT)


def make_trajectory(n: int = 10, slip: float = 0.05, torque: float = 0.1, sinkage: float | None = 0.003) -> list[dict]:
    return [
        {
            "t_s": i * 0.1,
            "x_m": i * 0.05,
            "y_m": 0.0,
            "z_m": 0.05,
            "roll_deg": 0.5,
            "pitch_deg": 1.0,
            "yaw_deg": 0.0,
            "v_forward_mps": 0.5,
            "mean_slip": slip,
            "mean_wheel_torque_nm": torque,
            "power_w": 0.05,
            "energy_j": 0.01 * (i + 1),
            "mean_sinkage_m": sinkage,
            "max_sinkage_m": sinkage * 1.5 if sinkage is not None else None,
        }
        for i in range(n)
    ]


def test_summarize_computes_distance_and_completion() -> None:
    trajectory = make_trajectory(n=10)
    summary = summarize(trajectory, rover_id="scout_v01", slope_deg=0.0, soil_material_id="loose_sand_scm_v0", wall_time_s=1.0)

    assert summary.distance_m == pytest.approx(0.45)
    assert summary.mean_slip == pytest.approx(0.05)
    assert summary.mean_sinkage_m == pytest.approx(0.003)
    assert summary.completed is True


def test_summarize_handles_missing_sinkage() -> None:
    trajectory = make_trajectory(n=5, sinkage=None)
    summary = summarize(trajectory, rover_id="scout_v01", slope_deg=0.0, soil_material_id="loose_sand_scm_v0", wall_time_s=1.0)

    assert summary.mean_sinkage_m is None
    assert summary.max_sinkage_m is None


def test_summarize_rejects_empty_trajectory() -> None:
    with pytest.raises(ValueError, match="empty trajectory"):
        summarize([], rover_id="scout_v01", slope_deg=0.0, soil_material_id="loose_sand_scm_v0", wall_time_s=1.0)


def _scout_summary() -> RunSummary:
    trajectory = make_trajectory(n=10, slip=0.04, torque=0.08, sinkage=0.004)
    return summarize(trajectory, rover_id="scout_v01", slope_deg=10.0, soil_material_id="loose_sand_scm_v0", wall_time_s=1.0)


def test_identity_baseline_copies_scout_metrics() -> None:
    scout_summary = _scout_summary()
    prediction = identity_baseline(scout_summary)

    assert prediction.status == "OK"
    assert prediction.metrics["mean_slip"] == scout_summary.mean_slip
    assert prediction.metrics["energy_j"] == scout_summary.energy_j


def test_slope_only_scales_torque_by_mass_ratio_not_slip() -> None:
    scout_rover = rover_registry().load("scout_v01")
    main_rover = rover_registry().load("main_v01")
    scout_summary = _scout_summary()

    prediction = slope_only(scout_summary, scout_rover, main_rover, slope_deg=10.0)

    assert prediction.status == "OK"
    assert prediction.metrics["mean_slip"] == scout_summary.mean_slip  # unmodified, as documented
    # main is heavier -> predicted torque should scale up
    assert prediction.metrics["mean_wheel_torque_nm"] > scout_summary.mean_wheel_torque_nm


def test_slope_and_soil_reuses_mobility_physics_and_clips_slip() -> None:
    scout_rover = rover_registry().load("scout_v01")
    main_rover = rover_registry().load("main_v01")
    scout_summary = _scout_summary()

    prediction = slope_and_soil(
        scout_summary,
        scout_rover,
        main_rover,
        slope_deg=10.0,
        soil_mohr_friction_angle_deg=28.0,
        soil_janosi_shear_m=0.02,
    )

    assert prediction.status == "OK"
    assert 0.0 <= prediction.metrics["mean_slip"] <= 1.0
    assert prediction.metrics["mean_sinkage_m"] >= 0.0


def test_user_formula_is_not_configured() -> None:
    prediction = user_formula()
    assert prediction.status == "NOT_CONFIGURED"
    assert prediction.metrics is None


def test_evaluate_flags_not_generalization_validated() -> None:
    main_summary = summarize(
        make_trajectory(n=10, slip=0.06, torque=0.15, sinkage=0.005),
        rover_id="main_v01",
        slope_deg=10.0,
        soil_material_id="loose_sand_scm_v0",
        wall_time_s=1.0,
    )
    scout_summary = _scout_summary()
    predictions = [identity_baseline(scout_summary), user_formula()]

    result = evaluate(main_summary, predictions)

    assert result.generalization_validated is False
    assert "not" in result.summary_note.lower() or "single" in result.summary_note.lower()
    identity_eval = next(p for p in result.predictors if p.predictor_name == "identity_baseline")
    assert identity_eval.status == "OK"
    assert identity_eval.errors["mean_slip"] == pytest.approx(abs(scout_summary.mean_slip - main_summary.mean_slip))
    not_configured_eval = next(p for p in result.predictors if p.predictor_name == "user_formula")
    assert not_configured_eval.status == "NOT_CONFIGURED"
    assert not_configured_eval.errors is None


def _pychrono_vehicle_importable_in_subprocess(timeout_s: float = 15.0) -> tuple[bool, str]:
    """Checks `import pychrono.vehicle` in a fresh subprocess with a hard timeout.

    Do this out-of-process: `import pychrono.vehicle` has been observed to not
    just fail (~4s DLL init error) but to hang for 60s+ when attempted after
    other Chrono objects have already been created in the same long-lived
    process (e.g. after earlier tests in this session ran ChSystemNSC/rover
    builds). An in-process try/except is not safe against that -- it can
    stall the whole test run. See docs/ENVIRONMENT_SETUP.md.
    """
    import subprocess
    import sys as _sys

    try:
        completed = subprocess.run(
            [_sys.executable, "-c", "import pychrono.vehicle"],
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        return False, f"pychrono.vehicle import hung past {timeout_s:.0f}s in a fresh subprocess"
    if completed.returncode != 0:
        return False, (completed.stderr or completed.stdout or "non-zero exit").strip().splitlines()[-1]
    return True, ""


@pytest.mark.pychrono
def test_scm_pilot_runs_end_to_end_scout_flat_loose() -> None:
    import os

    # Opt-in only: the subprocess-isolated check below has still been observed
    # to hang past its own timeout on this machine's broken pychrono.vehicle
    # install (not just fail cleanly) -- see docs/ENVIRONMENT_SETUP.md. Skip by
    # default so a normal `pytest -m pychrono` run can never stall on this.
    if os.environ.get("RUN_SCM_PILOT_E2E") != "1":
        pytest.skip(
            "scm_pilot end-to-end test is opt-in only (set RUN_SCM_PILOT_E2E=1) -- "
            "pychrono.vehicle has been observed to hang, not just fail, in this environment. "
            "See docs/ENVIRONMENT_SETUP.md."
        )

    from src.chrono.availability import get_pychrono_availability

    if not get_pychrono_availability().pychrono_available:
        pytest.skip("pychrono not available")

    ok, reason = _pychrono_vehicle_importable_in_subprocess()
    if not ok:
        pytest.skip(f"pychrono.vehicle not usable in this environment ({reason}); see docs/ENVIRONMENT_SETUP.md.")

    from dataclasses import replace

    from src.experiments.scm_pilot.presets import DEFAULT_COMMAND
    from src.experiments.scm_pilot.runner import run_pilot

    quick_command = replace(DEFAULT_COMMAND, duration_s=1.0, settle_s=0.2, ramp_s=0.2)
    result = run_pilot("scout", slope_deg=0.0, soil_key="loose", command=quick_command)

    assert result.trajectory
    assert result.summary.rover_id == "scout_v01"
