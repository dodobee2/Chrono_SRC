from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import pandas as pd
import streamlit as st

from src.backends import make_backend
from src.chrono.availability import get_pychrono_availability
from src.integration_schemas import RoverSpec, SimulationResult
from src.registries import (
    ContactPairRegistry,
    ControlProfileRegistry,
    ObservationRegistry,
    RoverRegistry,
    TerrainMaterialRegistry,
    TerrainRegistry,
)
from src.risk_fusion import analyze_dataframe
from src.sample_generator import save_sample_csv
from src.schemas import DEFAULT_MAIN_ROVER, DEFAULT_SCOUT_REFERENCE, MainRoverConfig, ScoutReferenceConfig


APP_DIR = Path(__file__).resolve().parent
SAMPLE_CSV = APP_DIR / "data" / "sample_patches.csv"
EXPERIMENT_OUTPUT_DIR = APP_DIR / "data" / "experiment_results"
ROVER_REGISTRY = RoverRegistry(APP_DIR / "rover_models", repo_root=APP_DIR)
TERRAIN_REGISTRY = TerrainRegistry(APP_DIR / "terrain_scenarios", repo_root=APP_DIR)
CONTROL_REGISTRY = ControlProfileRegistry(APP_DIR / "control_profiles", repo_root=APP_DIR)
OBSERVATION_REGISTRY = ObservationRegistry(APP_DIR / "observations", repo_root=APP_DIR)
TERRAIN_MATERIAL_REGISTRY = TerrainMaterialRegistry(APP_DIR / "terrain_materials", repo_root=APP_DIR)
CONTACT_PAIR_REGISTRY = ContactPairRegistry(APP_DIR / "contact_pairs", repo_root=APP_DIR)


GRADE_COLORS = {
    "Safe": "#2ca25f",
    "Caution": "#f1c40f",
    "Risk": "#de2d26",
    "Unknown": "#9e9e9e",
    "NOT_EVALUATED": "#9e9e9e",
    "GEOMETRY_ONLY": "#3182bd",
}


def ensure_sample_csv() -> None:
    if not SAMPLE_CSV.exists():
        save_sample_csv(SAMPLE_CSV)


def save_simulation_result(result: SimulationResult) -> Path:
    EXPERIMENT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = EXPERIMENT_OUTPUT_DIR / f"{result.experiment_id}.json"
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(result.to_dict(), handle, indent=2)
    return output_path


def launch_irrlicht_smoke_viewer(duration_s: float = 10.0) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "src.chrono.irrlicht_smoke_viewer",
        "--duration",
        f"{duration_s:.3f}",
    ]
    creationflags = subprocess.CREATE_NEW_CONSOLE if hasattr(subprocess, "CREATE_NEW_CONSOLE") else 0
    subprocess.Popen(command, cwd=APP_DIR, creationflags=creationflags)
    return command


def render_integration_usage_guide() -> None:
    with st.expander("How to use this tab", expanded=True):
        st.markdown(
            """
1. Select `RoverSpec`, `TerrainScenario`, and `ControlProfile` first. These are handoff YAML inputs.
2. Select `ScoutObservation` and `ContactPairSpec` when matching files exist. If left empty, the heuristic backend may use legacy fallback values.
3. Use `Run Experiment` for rover-risk contract output. Choose `heuristic` for an evaluated risk result, or `mock_chrono` for a non-evaluated integration placeholder.
4. Use `Run PyChrono Smoke` only to verify the local PyChrono environment. It drops a 1 kg box onto a fixed floor and must not be interpreted as rover risk.
5. Use `Open Irrlicht Smoke Viewer` when you want to see the same smoke scenario in a native PyChrono/Irrlicht window.

Expected smoke success signs: `status=completed`, `contact_detected=True`, `max_contact_count >= 1`, and `final_z < 0.25`.
            """.strip()
        )


def render_pychrono_environment() -> None:
    availability = get_pychrono_availability()
    st.subheader("PyChrono Environment")
    cols = st.columns(5)
    cols[0].metric("PyChrono", "Available" if availability.pychrono_available else "Unavailable")
    cols[1].metric("Vehicle module", "Available" if availability.vehicle_module_available else "Unavailable")
    cols[2].metric("Irrlicht module", "Available" if availability.irrlicht_module_available else "Unavailable")
    cols[3].metric("Version", availability.version or "unknown")
    cols[4].metric("Python", Path(availability.python_executable).name)
    st.caption(f"Python executable: {availability.python_executable}")
    st.caption(f"Module path: {availability.pychrono_module_path or '-'}")
    if availability.pychrono_available:
        st.success(availability.diagnostic_message)
    else:
        st.warning(availability.diagnostic_message)

    with st.expander("Irrlicht visualization", expanded=False):
        st.write(
            "The smoke simulation is headless by default. Irrlicht opens as a separate native PyChrono window; "
            "it is not embedded inside Streamlit. Close the Irrlicht window to stop the viewer."
        )
        viewer_duration = st.number_input("Viewer duration (s)", min_value=1.0, max_value=60.0, value=10.0, step=1.0)
        disabled = not availability.pychrono_available or not availability.irrlicht_module_available
        if st.button("Open Irrlicht Smoke Viewer", disabled=disabled):
            try:
                command = launch_irrlicht_smoke_viewer(viewer_duration)
                st.success("Irrlicht viewer launched in a separate window.")
                st.code(" ".join(command))
            except Exception as exc:
                st.error(f"Failed to launch Irrlicht viewer: {type(exc).__name__}: {exc}")
        st.code('conda activate chrono\ncd /d "C:\\K_SRC\\mobility_twin_mvp"\npython -m src.chrono.irrlicht_smoke_viewer --duration 10')


def render_simulation_result(result: SimulationResult, path: str | None = None) -> None:
    if result.backend_name == "mock_chrono":
        st.error(
            "MOCK CHRONO -- Chrono physics engine was not executed. "
            "Reference values are heuristic artifacts, not real Chrono results."
        )
    if result.backend_name == "pychrono_smoke":
        st.info("PyChrono Smoke is a 1 kg box-drop environment check. It does not calculate rover mobility risk.")

    st.subheader("SimulationResult")
    metric_cols = st.columns(5)
    metric_cols[0].metric("Backend", result.backend_name)
    metric_cols[1].metric("model_status", result.model_status)
    metric_cols[2].metric("evaluation_state", result.evaluation_state)
    metric_cols[3].metric("Grade", result.grade)
    metric_cols[4].metric("Risk", "N/A" if result.final_risk is None else f"{result.final_risk:.2f}")

    if path:
        st.caption(f"Saved JSON: {path}")

    left, right = st.columns(2)
    with left:
        st.write("Risk components")
        st.dataframe(pd.Series(result.risk_components, name="risk").to_frame(), use_container_width=True)
    with right:
        st.write("Typed metrics")
        st.dataframe(pd.Series(result.metrics_typed.to_dict(), name="value").to_frame(), use_container_width=True)

    trajectory_csv = result.artifacts.get("trajectory_csv") if result.artifacts else None
    if trajectory_csv and Path(trajectory_csv).exists():
        trajectory = pd.read_csv(trajectory_csv)
        st.write("Trajectory")
        st.dataframe(trajectory, use_container_width=True)
        if "time_s" in trajectory and "position_z_m" in trajectory:
            st.line_chart(trajectory.set_index("time_s")["position_z_m"])

    if result.artifacts:
        st.write("Artifacts")
        st.json(result.artifacts)

    st.write("Full JSON")
    st.json(result.to_dict())


def sidebar_configs(rover_preset: RoverSpec | None = None) -> tuple[MainRoverConfig, ScoutReferenceConfig]:
    defaults = DEFAULT_MAIN_ROVER if rover_preset is None else rover_preset.to_main_config()
    st.sidebar.header("Main rover")
    main = MainRoverConfig(
        mass_kg=st.sidebar.number_input("Mass (kg)", min_value=1.0, value=defaults.mass_kg, step=1.0),
        wheel_radius_m=st.sidebar.number_input("Wheel radius (m)", min_value=0.01, value=defaults.wheel_radius_m, step=0.01),
        wheel_width_m=st.sidebar.number_input("Wheel width (m)", min_value=0.01, value=defaults.wheel_width_m, step=0.01),
        wheelbase_m=st.sidebar.number_input("Wheelbase (m)", min_value=0.05, value=defaults.wheelbase_m, step=0.05),
        track_width_m=st.sidebar.number_input("Track width (m)", min_value=0.05, value=defaults.track_width_m, step=0.05),
        cg_height_m=st.sidebar.number_input("CG height (m)", min_value=0.02, value=defaults.cg_height_m, step=0.01),
        ground_clearance_m=st.sidebar.number_input(
            "Ground clearance (m)", min_value=0.01, value=defaults.ground_clearance_m, step=0.01
        ),
        driven_wheel_count=st.sidebar.number_input(
            "Driven wheel count", min_value=1, max_value=12, value=defaults.driven_wheel_count, step=1
        ),
        max_wheel_torque_nm=st.sidebar.number_input(
            "Max wheel torque (Nm)", min_value=0.1, value=defaults.max_wheel_torque_nm, step=1.0
        ),
        mu_eff=st.sidebar.number_input("Effective mu", min_value=0.01, max_value=2.0, value=defaults.mu_eff, step=0.05),
        crr=st.sidebar.number_input("Rolling resistance Crr", min_value=0.0, max_value=1.0, value=defaults.crr, step=0.01),
    )

    st.sidebar.header("Scout reference")
    scout = ScoutReferenceConfig(
        mass_kg=st.sidebar.number_input("Scout mass (kg)", min_value=0.1, value=DEFAULT_SCOUT_REFERENCE.mass_kg, step=0.5),
        wheel_radius_m=st.sidebar.number_input(
            "Scout wheel radius (m)", min_value=0.005, value=DEFAULT_SCOUT_REFERENCE.wheel_radius_m, step=0.005
        ),
        wheel_width_m=st.sidebar.number_input(
            "Scout wheel width (m)", min_value=0.005, value=DEFAULT_SCOUT_REFERENCE.wheel_width_m, step=0.005
        ),
        wheel_load_n=st.sidebar.number_input(
            "Scout wheel load (N)", min_value=0.1, value=DEFAULT_SCOUT_REFERENCE.wheel_load_n, step=1.0
        ),
    )

    if st.sidebar.button("Regenerate sample CSV"):
        save_sample_csv(SAMPLE_CSV)
        st.sidebar.success("sample_patches.csv regenerated")

    return main, scout


def render_integration_experiment() -> None:
    st.caption("Integration Contract MVP: RoverSpec + TerrainScenario + ScoutObservation + ContactPairSpec + ControlProfile -> SimulationResult")
    render_integration_usage_guide()
    render_pychrono_environment()

    rover_ids = ROVER_REGISTRY.ids()
    terrain_ids = TERRAIN_REGISTRY.ids()
    control_ids = CONTROL_REGISTRY.ids()
    if not rover_ids or not terrain_ids or not control_ids:
        st.warning("Registry YAML files are missing. Check rover_models, terrain_scenarios, and control_profiles.")
        return

    col1, col2, col3 = st.columns(3)
    rover_id = col1.selectbox("RoverSpec", rover_ids)
    terrain_id = col2.selectbox("TerrainScenario", terrain_ids)
    control_id = col3.selectbox("ControlProfile", control_ids)

    rover = ROVER_REGISTRY.load(rover_id)
    terrain = TERRAIN_REGISTRY.load(terrain_id)
    control = CONTROL_REGISTRY.load(control_id)
    st.session_state["selected_rover_id"] = rover_id

    observation_matches = OBSERVATION_REGISTRY.ids_for(terrain_id=terrain_id, control_profile_id=control_id)
    observation_options = ["<none: use legacy if available>"] + observation_matches
    contact_matches = CONTACT_PAIR_REGISTRY.ids_for(rover.wheel_material_id, terrain.material_id)
    contact_options = ["<none: rover fallback>"] + contact_matches

    col4, col5, col6 = st.columns(3)
    observation_id = col4.selectbox("ScoutObservation", observation_options)
    contact_pair_id = col5.selectbox("ContactPairSpec", contact_options)
    backend_id = col6.selectbox("Mock/heuristic backend", ["heuristic", "mock_chrono"])

    observation = None if observation_id.startswith("<none") else OBSERVATION_REGISTRY.load(observation_id)
    contact_pair = None if contact_pair_id.startswith("<none") else CONTACT_PAIR_REGISTRY.load(contact_pair_id)
    if contact_pair is not None:
        CONTACT_PAIR_REGISTRY.validate_references(contact_pair, rover, terrain, TERRAIN_MATERIAL_REGISTRY)

    material = TERRAIN_MATERIAL_REGISTRY.load(terrain.material_id) if terrain.material_id in TERRAIN_MATERIAL_REGISTRY.ids() else None
    if material and material.parameter_source == "assumed":
        st.info(f"Terrain material `{material.material_id}` uses assumed parameters, confidence={material.confidence:.2f}.")
    if contact_pair and contact_pair.source == "assumed":
        st.info(f"Contact pair `{contact_pair.contact_pair_id}` uses assumed parameters, confidence={contact_pair.confidence:.2f}.")

    with st.expander("Selected handoff schemas", expanded=False):
        tabs = st.tabs(["RoverSpec", "TerrainScenario", "ScoutObservation", "ContactPairSpec", "ControlProfile"])
        tabs[0].json(rover.to_dict())
        tabs[1].json(terrain.to_dict())
        tabs[2].json({} if observation is None else observation.to_dict())
        tabs[3].json({} if contact_pair is None else contact_pair.to_dict())
        tabs[4].json(control.to_dict())

    col_run_1, col_run_2 = st.columns(2)
    if col_run_1.button("Run Experiment", type="primary"):
        backend = make_backend(backend_id)
        result = backend.run(rover, terrain, control, observation=observation, contact_pair=contact_pair)
        output_path = save_simulation_result(result)
        st.session_state["last_simulation_result"] = result
        st.session_state["last_simulation_result_path"] = str(output_path)

    if col_run_2.button("Run PyChrono Smoke"):
        with st.status("RUNNING", expanded=True) as status:
            backend = make_backend("pychrono_smoke")
            result = backend.run(rover, terrain, control, observation=observation, contact_pair=contact_pair)
            output_path = save_simulation_result(result)
            st.session_state["last_simulation_result"] = result
            st.session_state["last_simulation_result_path"] = str(output_path)
            if result.status == "completed":
                status.update(label="COMPLETED", state="complete")
            elif result.status == "timeout":
                status.update(label="TIMEOUT", state="error")
            else:
                status.update(label="FAILED", state="error")

    result = st.session_state.get("last_simulation_result")
    if result:
        render_simulation_result(result, st.session_state.get("last_simulation_result_path"))


def plot_traversability_map(results: pd.DataFrame) -> plt.Figure:
    fig, ax = plt.subplots(figsize=(8, 5))
    for _, row in results.iterrows():
        x = int(row["grid_x"])
        y = int(row["grid_y"])
        grade = str(row["grade"])
        color = GRADE_COLORS.get(grade, GRADE_COLORS["Unknown"])
        rect = patches.Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor=color, edgecolor="white", linewidth=2)
        ax.add_patch(rect)
        ax.text(x, y, f'{int(row["risk_score"])}', ha="center", va="center", color="black", fontsize=11, weight="bold")

    ax.set_xlim(results["grid_x"].min() - 0.5, results["grid_x"].max() + 0.5)
    ax.set_ylim(results["grid_y"].min() - 0.5, results["grid_y"].max() + 0.5)
    ax.set_xticks(sorted(results["grid_x"].unique()))
    ax.set_yticks(sorted(results["grid_y"].unique()))
    ax.set_xlabel("Grid X")
    ax.set_ylabel("Grid Y")
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_title("Main-Rover Mobility Risk Map (Rover-specific Traversability Layer)")
    return fig


def render_summary(results: pd.DataFrame) -> None:
    terrain_counts = results["terrain_label"].value_counts().rename_axis("terrain_label").reset_index(name="patch_count")
    st.subheader("Terrain classification counts")
    st.dataframe(terrain_counts, use_container_width=True, hide_index=True)

    safe_count = int((results["grade"] == "Safe").sum())
    caution_count = int((results["grade"] == "Caution").sum())
    risk_count = int((results["grade"] == "Risk").sum())
    avg_risk = float(results["final_risk"].mean())
    worst = results.sort_values("final_risk", ascending=False).iloc[0]

    col1, col2, col3, col4, col5 = st.columns(5)
    col1.metric("Safe", safe_count)
    col2.metric("Caution", caution_count)
    col3.metric("Risk", risk_count)
    col4.metric("Average risk", f"{avg_risk:.2f}")
    col5.metric("Highest-risk patch", f'#{int(worst["patch_id"])}', f'{int(worst["risk_score"])}')


def render_detail(results: pd.DataFrame) -> None:
    st.subheader("Patch detail")
    selected_patch = st.selectbox("Patch", results["patch_id"].astype(int).tolist())
    row = results.loc[results["patch_id"] == selected_patch].iloc[0]

    left, right = st.columns(2)
    with left:
        st.write("Measurement")
        measurement_cols = [
            "slope_long_deg",
            "slope_lat_deg",
            "roughness_m",
            "obstacle_height_m",
            "gap_width_m",
            "scout_slip",
            "scout_sinkage_m",
            "scout_wheel_torque_nm",
            "scout_cot",
            "vibration_rms_g",
            "surface_hint",
        ]
        st.dataframe(row[measurement_cols].to_frame("value"), use_container_width=True)
    with right:
        st.write("Result")
        result_values = {
            "terrain_label": row["terrain_label"],
            "terrain_reason": row["terrain_reason"],
            "F_req_N": f'{row["physics_f_req_n"]:.2f}',
            "F_avail_N": f'{row["physics_f_avail_n"]:.2f}',
            "traction_margin_N": f'{row["physics_traction_margin_n"]:.2f}',
            "tipover_margin_deg": f'{row["physics_tipover_margin_deg"]:.2f}',
            "obstacle_ratio": f'{row["physics_obstacle_ratio"]:.2f}',
            "predicted_slip": f'{row["physics_predicted_main_slip"]:.2f}',
            "predicted_sinkage_m": f'{row["physics_predicted_main_sinkage_m"]:.3f}',
            "final_risk": f'{row["final_risk"]:.2f}',
            "grade": row["grade"],
            "hard_failure_reasons": row["hard_failure_reasons"] or "-",
        }
        st.dataframe(pd.Series(result_values, name="value").to_frame(), use_container_width=True)


def render_legacy_risk_map() -> None:
    st.info("This legacy screen is separate from Integration Experiment. It is the original concept-validation heuristic map.")
    rover_preset = None
    selected_rover_id = st.session_state.get("selected_rover_id")
    if selected_rover_id and st.checkbox("Use selected RoverSpec as MainRoverConfig defaults"):
        rover_preset = ROVER_REGISTRY.load(selected_rover_id)

    main_config, scout_reference = sidebar_configs(rover_preset)
    source = pd.read_csv(SAMPLE_CSV)
    edited = st.data_editor(source, use_container_width=True, num_rows="dynamic")

    if st.button("Calculate patch risks", type="primary") or "last_results" not in st.session_state:
        st.session_state["last_results"] = analyze_dataframe(edited, main_config, scout_reference)

    results = st.session_state["last_results"]
    st.subheader("Patch risk table")
    st.dataframe(
        results[["patch_id", "grid_x", "grid_y", "terrain_label", "risk_score", "grade", "hard_failure_reasons"]],
        use_container_width=True,
        hide_index=True,
    )

    render_summary(results)

    st.subheader("Main-Rover Mobility Risk Map (Rover-specific Traversability Layer)")
    st.pyplot(plot_traversability_map(results))

    render_detail(results)


def main() -> None:
    st.set_page_config(page_title="Scout-to-Main Rover Mobility Twin MVP", layout="wide")
    ensure_sample_csv()

    st.title("Scout-to-Main Rover Mobility Twin MVP")
    st.write(
        "Scout rover observations are converted into main-rover-specific traversability estimates "
        "through the Integration Contract MVP."
    )
    st.caption("Final output: Main-Rover Mobility Risk Map (Rover-specific Traversability Layer).")

    tab_integration, tab_legacy = st.tabs(["Integration Experiment", "Legacy Heuristic Risk Map"])
    with tab_integration:
        render_integration_experiment()
    with tab_legacy:
        render_legacy_risk_map()


if __name__ == "__main__":
    main()
