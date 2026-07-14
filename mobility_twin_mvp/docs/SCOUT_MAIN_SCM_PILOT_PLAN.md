# Scout-to-Main SCM Transfer Pilot

## Why this exists

Wiring rover+terrain+SCM directly into the main Contract v2 app (Streamlit,
risk map, full 5-zone arena) all at once was judged too much complexity too
soon -- it risks spending the next round of work debugging integration
plumbing instead of answering the actual research question. This pilot is a
small, standalone experiment, decoupled from the main app, that asks exactly
one question (see 우주로버 아이디어.pdf, 01 Executive Summary):

> Does scout_v01's measured slip/sinkage/torque/energy response on an SCM
> soil patch let us predict main_v01's response on the same patch better
> than a slope-only or identity baseline?

## Structure

```
mobility_twin_mvp/
  src/experiments/scm_pilot/
    presets.py     slope/soil/rover/command presets
    scenario.py     builds one (rover, slope, soil) Chrono scenario
    runner.py       drives one headless straight-cruise run, logs a trajectory
    metrics.py      trajectory -> RunSummary
    predictor.py    identity_baseline / slope_only / slope_and_soil / user_formula
    evaluator.py    compares a baseline's prediction against Main ground truth
  scripts/run_scm_pilot.py   CLI entry point
  data/scm_pilot/<run_id>/   CSV + JSON output
```

No `terrain_models/` or `rover_models/hojin_v01`-style parallel directories
were created. This pilot reuses what already exists and is already tested:

- `rover_models/scout_v01/rover.yaml`, `rover_models/main_v01/rover.yaml` --
  Hojin's real, Chrono-verified rover specs (see
  `handoff/rover_module_v01/02_설계_이유.md`)
- `src/chrono/rover_factory.py` -- Contract v2 `RoverSpec` -> Chrono rover
  (already has the collision-system fix and cg_height_m/cg_xyz_m translation)
- `src/chrono/terrain_factory.py::build_scm_terrain` -- `TerrainMaterialSpec`
  -> `SCMTerrain`
- `terrain_materials/loose_sand_scm_v0.yaml` (from `handoff/map.py`),
  `medium_soil_scm_v0.yaml`, `firm_soil_scm_v0.yaml` (pilot assumptions,
  clearly labeled `parameter_source: assumed`)
- `src/mobility_physics.py::predict_main_slip`/`predict_main_sinkage_m` --
  reused inside the `slope_and_soil` baseline instead of inventing new
  scaling math

A second Chrono/terrain integration attempt was started independently in
parallel (a `terrain_models/jongmin_v01/scm_patch.py`-based design) and
briefly overwrote these files before being stopped; it is not present on
disk. If it resumes later, reconcile against this plan first rather than
maintaining two rover/terrain representations side by side.

## Scope (first pass)

- rovers: `scout_v01`, `main_v01` only
- slope: flat (0 deg), 10 deg -- applied by tilting gravity, not the terrain
  mesh (see `scenario.py` docstring for why, and its real limitation:
  longitudinal-only, no lateral tip-over or approach geometry)
- soil: loose / medium / firm SCM presets
- command: one straight, ramped cruise command (no turning, no waypoints)
- headless only, no Streamlit, no arena, no rock/gate/obstacle zones

## Baselines (deliberately not oversold)

| predictor | what it does |
|---|---|
| `identity_baseline` | copies scout's measured response onto main verbatim |
| `slope_only` | scales torque/energy by a mass*slope force-balance ratio only (crr=0); slip/sinkage copied unmodified |
| `slope_and_soil` | reuses `mobility_physics.py`'s pressure/geometry scaling for slip/sinkage; soil only enters via the crr term in the torque/energy ratio (Bekker Kphi/Kc are not consumed by the slip/sinkage formulas -- a known limitation inherited from the existing heuristic module) |
| `user_formula` | `NOT_CONFIGURED` until a real calibration pass exists -- no guessed coefficients |

## Success criteria for this pilot

1. Scout and Main both run under the same SCM condition.
2. Scout metrics and Main ground truth are saved to CSV/JSON
   (`data/scm_pilot/<run_id>/{scout,main}_trajectory.csv`, `result.json`).
3. Baseline predictions are compared against Main ground truth.
4. slip/sinkage/torque/energy/distance/completion errors are produced.
5. The evaluator explicitly states this is **not** a validated
   generalization (`PilotEvaluation.generalization_validated = False`,
   with a summary note) -- one datapoint is not a calibration campaign.

## Current status (2026-07-14): skeleton only, not run end-to-end

`pychrono.vehicle` (and therefore `SCMTerrain`) fails to import in this
project's verified `chrono` conda env with a DLL init error. Confirmed this
is not a PATH issue (`os.add_dll_directory` on `Library/bin` does not fix
it). Worse: a fresh `import pychrono.vehicle` fails cleanly in ~4s in a new
process, but has been observed to **hang for 60s+** (not just fail) when
attempted in a longer-lived process that already built other Chrono objects
(e.g. inside a pytest session after earlier rover-builder tests ran). Because
of that, `tests/test_scm_pilot.py`'s end-to-end test is opt-in only
(`RUN_SCM_PILOT_E2E=1`) and even then checks importability in a fresh
subprocess with a hard timeout first, rather than importing in-process --
see that test's docstring. Do not remove that guard without first confirming
the hang is actually fixed.

Everything that does not require `pychrono.vehicle` is implemented and
tested: presets, `RunSummary` extraction, all four baselines (reusing
`mobility_physics.py`), and the evaluator's error/summary output --
`tests/test_scm_pilot.py` covers all of it without needing a working
PyChrono install.

## Next step once pychrono.vehicle is fixed

```
conda activate chrono
cd mobility_twin_mvp
python scripts/run_scm_pilot.py --slope flat --soil loose
```

First confirm `ChSystemNSC` actually works with `SCMTerrain` (see
`scenario.py` docstring -- the vendored rover builder hardcodes NSC contact
materials, but `handoff/map.py`'s arena uses `ChSystemSMC`; this combination
has never been tested). If SCM requires SMC, switch `scenario.py` to
`ChSystemSMC` and update the vendored rover builder's contact materials to
match.
