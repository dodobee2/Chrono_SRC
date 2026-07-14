# Scout-to-Main Rover Mobility Twin MVP

Current stage: **Integration Contract MVP**

This project estimates main-rover mobility risk from scout terrain response data. It keeps the original CSV-based heuristic demo, while adding an Integration Contract v2 skeleton that can later receive real rover and terrain models without assuming CAD, collision models, SCM/DEM parameters, or a PyChrono build today.

Final output name:

**Main-Rover Mobility Risk Map (Rover-specific Traversability Layer)**

## Install

```bash
cd mobility_twin_mvp
pip install -r requirements.txt
```

## Run

```bash
pytest -q
python -m compileall src
streamlit run app.py --browser.gatherUsageStats false
```

## Terms

- **Scout Terrain Response / Observation**: scout rover measurements and traversal response, stored as `ScoutObservation`.
- **Terrain Scenario**: environment geometry, material reference, obstacles, dimensions, frame, and random seed.
- **Terrain Material**: terrain material/contact assumptions or measured parameters.
- **Contact Pair**: effective wheel-terrain contact parameters for a wheel material and terrain material.
- **Main-Rover Mobility Risk**: rover-specific risk evaluated from a selected rover, terrain, control, observation, and backend.
- **Rover-specific Traversability Layer**: the map layer produced for a particular main rover configuration.

## Integration Contract V2

The integration path is:

```text
RoverSpec
TerrainScenario
ScoutObservation
TerrainMaterialSpec
ContactPairSpec
ControlProfile
  -> MobilityBackend
  -> SimulationResult
  -> Main-Rover Mobility Risk Map
```

Backends:

- `heuristic`: wraps the existing MVP equations. It requires `ScoutObservation` or a deprecated legacy `TerrainScenario.scout_response`.
- `mock_chrono`: does not run Chrono physics. It returns `evaluation_state=NOT_EVALUATED`, `final_risk=None`, and stores heuristic comparison values only under `artifacts`.
- `pychrono_smoke`: runs a real headless PyChrono box-drop smoke scenario when PyChrono is importable in the active Python environment. It does not evaluate rover mobility risk.

## PyChrono Smoke Integration

PyChrono is not a `requirements.txt` dependency; it must be installed via conda
in a separate environment. See [docs/ENVIRONMENT_SETUP.md](docs/ENVIRONMENT_SETUP.md)
for the verified environment, install steps, and the collision-system setup
note.

The Streamlit `Integration Experiment` tab includes a `PyChrono Environment` panel:

- Python executable
- PyChrono availability
- `pychrono.vehicle` availability
- module path
- version when discoverable
- diagnostic message

The `Run PyChrono Smoke` button calls a pure Python backend path:

```text
app.py
  -> src.backends.make_backend("pychrono_smoke")
  -> src.chrono.pychrono_backend.PyChronoSmokeBackend
  -> python -m src.chrono.pychrono_runner
  -> src.chrono.smoke_scenario.run_smoke_scenario()
```

The smoke scenario uses:

- `ChSystemNSC` when available, otherwise a compatible current PyChrono contact system if present
- gravity `[0, 0, -9.81]`
- fixed rigid floor
- 1 kg box
- initial box position `[0, 0, 1]`
- duration `3 s`
- step size `0.001 s`
- no visualization

Outputs are written under `data/chrono_smoke/` when the smoke run completes:

- `trajectory.csv`
- `result.json`
- `runner.log`

Optional Irrlicht viewer:

```bat
conda activate chrono
cd /d "C:\K_SRC\mobility_twin_mvp"
python -m src.chrono.irrlicht_smoke_viewer --duration 10
```

The Irrlicht viewer opens a separate native PyChrono window. It is a visual smoke check for the box-drop scenario, not an embedded Streamlit panel and not a rover-risk evaluation.

The returned `SimulationResult` always uses:

```text
model_status = chrono_smoke
evaluation_state = NOT_EVALUATED
final_risk = None
grade = NOT_EVALUATED
```

This smoke result is not included in Safe/Caution/Risk statistics.

## Handoff Files

Rover model team:

```text
rover_models/<rover_id>/rover.yaml
rover_models/_template/rover.yaml
```

Terrain model team:

```text
terrain_scenarios/<terrain_id>/terrain.yaml
terrain_scenarios/_template/terrain.yaml
terrain_materials/<material_id>.yaml
terrain_materials/_template/material.yaml
contact_pairs/<contact_pair_id>.yaml
contact_pairs/_template/contact_pair.yaml
```

Scout observation / experiment team:

```text
observations/<observation_id>/observation.yaml
observations/_template/observation.yaml
control_profiles/<profile_id>.yaml
control_profiles/_template.yaml
```

Checklist:

```text
docs/HANDOFF_CHECKLIST.md
```

## Schema Summary

`RoverSpec`: `rover_id`, `display_name`, `mass_kg`, `wheel_radius_m`, `wheel_width_m`, `wheelbase_m`, `track_width_m`, `cg_height_m`, `ground_clearance_m`, `driven_wheel_count`, `max_wheel_torque_nm`, `wheel_material_id`, `wheel_contact_model`, `fallback_mu_eff`, `fallback_crr`, `model_uri`, `metadata`.

`TerrainScenario`: `terrain_id`, `display_name`, `terrain_type`, `surface_hint`, `geometry`, `material_id`, `dimensions_xyz_m`, `frame_id`, `random_seed`, `obstacles`, `slope_long_deg`, `slope_lat_deg`, `roughness_m`, `gap_width_m`, `patch_id`, `grid_x`, `grid_y`, `observation_state`, `geometry_confidence`, `prediction_confidence`, `legacy_scout_response`, `metadata`.

`ScoutObservation`: `observation_id`, `terrain_id`, `scout_rover_id`, `control_profile_id`, `timestamp_utc`, grid/pose fields, terrain geometry measurements, speed/slip/sinkage/torque/COT/vibration fields, confidence fields, `source_type`, `observation_state`, `metadata`.

`TerrainMaterialSpec`: `material_id`, `model_type`, `friction_nominal`, `rolling_resistance_nominal`, `restitution`, `scm_parameters`, `parameter_source`, `confidence`, `metadata`.

`ContactPairSpec`: `contact_pair_id`, `wheel_material_id`, `terrain_material_id`, `mu_eff`, `crr_eff`, `source`, `confidence`, `metadata`.

`SimulationResult`: legacy-compatible `metrics` dict plus typed `metrics_typed`, `prediction_confidence`, `model_status`, `evaluation_state`, `failure_reasons`, `final_risk`, `grade`, `artifacts`.

## Migration Rules

- Legacy `TerrainScenario.scout_response` is still read, but it is migrated to a temporary `ScoutObservation` with a deprecation warning.
- Legacy `RoverSpec.mu_eff` and `RoverSpec.crr` are still read, but they migrate to `fallback_mu_eff` and `fallback_crr`.
- Legacy obstacle fields `x_m`, `y_m`, `height_m`, `width_m`, `length_m` are read and migrated to `pose` and `dimensions_xyz_m`.
- Mock Chrono results are not counted as Safe/Caution/Risk because they are `NOT_EVALUATED`.

## Existing Heuristic Map

The Streamlit app has two tabs:

1. `Integration Experiment`
2. `Legacy Heuristic Risk Map`

The legacy tab is intentionally separate from Integration Contract v2. It still reads `data/sample_patches.csv`, computes patch risk, and renders the Main-Rover Mobility Risk Map using matplotlib.

## Physics Used By The Heuristic Backend

```text
F_req = m * g * sin(alpha) + Crr * m * g * cos(alpha)
F_torque = driven_wheel_count * max_wheel_torque / wheel_radius
F_friction = mu_eff * m * g * cos(alpha)
F_avail = min(F_torque, F_friction)
traction_margin_n = F_avail - F_req
traction_margin_ratio = (F_avail - F_req) / max(F_req, 1.0)
beta_crit_deg = degrees(atan((track_width / 2) / cg_height))
tipover_margin_deg = beta_crit_deg - abs(slope_lat_deg)
obstacle_ratio = obstacle_height / wheel_radius
gap_ratio = gap_width / (2 * wheel_radius)
clearance_ratio = obstacle_height / ground_clearance
```

Scout slip/sinkage scaling remains a concept-validation heuristic, not a verified final model.

## Next Steps

Next stage: **Chrono execution smoke integration**

After that: **Actual rover/terrain model integration**

The replacement point is `src/backends.py`, especially `MockChronoBackend`. A real `PyChronoBackend` should introduce:

- `rover_factory` ([src/chrono/rover_factory.py](src/chrono/rover_factory.py)): translates Contract v2 `RoverSpec`
  into the vendored `handoff/rover_module_v01` builder (Hojin, 2026-07-14) and builds real
  rigid-body rover + wheel motors in a `ChSystem`. Implemented and tested
  (`tests/test_handoff_integration.py`, `@pytest.mark.pychrono`) against `rover_models/scout_v01`
  and `rover_models/main_v01`.
- `terrain_factory` ([src/chrono/terrain_factory.py](src/chrono/terrain_factory.py)): translates `TerrainScenario` +
  `TerrainMaterialSpec` into Chrono terrain objects. Rigid-flat and SCM-granular are implemented and
  tested; rocky/uneven/gated/sloped terrain is NOT implemented -- see `handoff/map.py` (Jongmin,
  2026-07-14) for a reference 5-zone arena that still needs to be driven by `TerrainScenario`.
- `control_adapter`: `ControlProfile` to Chrono driver/controller input -- not started.
- `result_extractor`: Chrono pose, slip, sinkage, torque, contact, energy, rollover/stall events to `SimulationResult` -- not started.

Known gaps to close before a real `pychrono_physics` backend can be wired into `make_backend()`:

- `rover_models/main_rover_baseline` was reconciled (2026-07-14) to the same physical numbers as
  the real, Chrono-verified `rover_models/main_v01` (2.8 kg) -- it is kept only as the
  fallback_mu_eff/fallback_crr-populated fixture existing tests already wire through for
  `HeuristicBackend`; `main_v01` stays the canonical no-fabricated-friction entry for real Chrono
  work. Main's mass intentionally stays in the team's 1.5-3 kg policy range (not a
  larger/heavier-scale vehicle) -- see `handoff/rover_module_v01/02_설계_이유.md`.
- `pychrono.vehicle` (and therefore `SCMTerrain`) currently fails to import in the verified
  `chrono` conda env with a DLL load error (`DLL init routine could not be executed`), even though
  `pychrono` core imports fine and `importlib.util.find_spec("pychrono.vehicle")` reports the
  module as present. Confirmed this is not a PATH/DLL-search-directory issue (explicitly adding
  the env's `Library/bin` via `os.add_dll_directory` does not fix it, and the native DLLs there
  are present at plausible sizes) -- it looks like a real install/build issue in this `chrono` env
  that would need a `pychrono` reinstall to fix, not attempted yet. Not a `terrain_factory` bug --
  see [docs/ENVIRONMENT_SETUP.md](docs/ENVIRONMENT_SETUP.md). `tests/test_handoff_integration.py`
  skips the affected test with this diagnosis rather than failing silently.

No actual Chrono C++ build, rover CAD generation, or SCM/DEM parameter calibration is performed in this stage.

## Scout-to-Main SCM Pilot

A small, standalone experiment under `src/experiments/scm_pilot/` (see
[docs/SCOUT_MAIN_SCM_PILOT_PLAN.md](docs/SCOUT_MAIN_SCM_PILOT_PLAN.md)) asks one focused question --
does scout_v01's SCM soil response predict main_v01's response better than a slope-only or identity
baseline -- without touching the main Streamlit app or the full arena. It reuses `rover_factory.py`,
`terrain_factory.py`, and `mobility_physics.py` rather than building a second rover/terrain
representation. Skeleton implemented and unit-tested; end-to-end execution is blocked on the
`pychrono.vehicle` import failure above (`scripts/run_scm_pilot.py --slope flat --soil loose`).

## Rigid-Terrain Scout-to-Main Transfer Pilot

Since SCM is blocked, `src/experiments/rigid_transfer_pilot/` asks the same transfer question on
rigid terrain only (flat, 3 slopes, 3 friction levels, 2 obstacle heights -- 9 conditions), which
needs only `pychrono` core, not `pychrono.vehicle`. Reuses `rover_factory.py` and
`terrain_factory.py` unmodified; slope is applied by tilting gravity and the obstacle condition is
built with a small local helper, both to avoid touching the shared factories. Runs a torque-limited
straight command (no ideal wheel-speed motor) and compares four predictors (`identity_baseline`,
`slope_only`, `terrain_only`, `user_formula`) against real Main ground truth with MAE and rank
correlation. **Verified end-to-end today** (`python scripts/run_rigid_transfer_pilot.py`) -- but see
"Native loading is unpredictably slow" in
[docs/ENVIRONMENT_SETUP.md](docs/ENVIRONMENT_SETUP.md): Chrono scenario building in this environment
has a roughly 1-in-3-4 chance of hanging 60s+ per process start, unrelated to this pilot's code, so a
run may need to be killed and retried.

