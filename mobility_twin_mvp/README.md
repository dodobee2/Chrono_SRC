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

- `rover_factory`: Hojin rover spec and model URI to Chrono rover/wheel/contact objects
- `terrain_factory`: Jongmin terrain scenario to rigid/SCM/DEM/mesh/heightmap objects
- `control_adapter`: `ControlProfile` to Chrono driver/controller input
- `result_extractor`: Chrono pose, slip, sinkage, torque, contact, energy, rollover/stall events to `SimulationResult`

No actual PyChrono install, Chrono C++ build, rover CAD generation, collision model invention, or SCM/DEM parameter identification is performed in this stage.
