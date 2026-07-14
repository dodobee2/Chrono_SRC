from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import pandas as pd
import streamlit as st

from src.backends import HeuristicBackend, make_backend
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



def main_config_from_rover_spec(rover: RoverSpec) -> MainRoverConfig:
    """Use RoverSpec values in the legacy heuristic map."""
    return MainRoverConfig(
        mass_kg=rover.mass_kg,
        wheel_radius_m=rover.wheel_radius_m,
        wheel_width_m=rover.wheel_width_m,
        wheelbase_m=rover.wheelbase_m,
        track_width_m=rover.track_width_m,
        cg_height_m=rover.cg_height_m,
        ground_clearance_m=rover.ground_clearance_m,
        driven_wheel_count=rover.driven_wheel_count,
        max_wheel_torque_nm=rover.max_wheel_torque_nm,
        mu_eff=rover.fallback_mu_eff if rover.fallback_mu_eff is not None else DEFAULT_MAIN_ROVER.mu_eff,
        crr=rover.fallback_crr if rover.fallback_crr is not None else DEFAULT_MAIN_ROVER.crr,
    )


def scout_reference_from_rover_spec(rover: RoverSpec) -> ScoutReferenceConfig:
    wheel_count = max(int(rover.metadata.get("wheel_count", rover.driven_wheel_count or 4)), 1)
    return ScoutReferenceConfig(
        mass_kg=rover.mass_kg,
        wheel_radius_m=rover.wheel_radius_m,
        wheel_width_m=rover.wheel_width_m,
        wheel_load_n=rover.mass_kg * 9.81 / wheel_count,
    )


def rover_parameter_comparison_frame(main_rover: RoverSpec, scout_rover: RoverSpec) -> pd.DataFrame:
    rows = {
        "질량 kg": {"정찰로버": scout_rover.mass_kg, "메인로버": main_rover.mass_kg},
        "바퀴 반지름 m": {"정찰로버": scout_rover.wheel_radius_m, "메인로버": main_rover.wheel_radius_m},
        "바퀴 폭 m": {"정찰로버": scout_rover.wheel_width_m, "메인로버": main_rover.wheel_width_m},
        "휠베이스 m": {"정찰로버": scout_rover.wheelbase_m, "메인로버": main_rover.wheelbase_m},
        "좌우 바퀴 간격 m": {"정찰로버": scout_rover.track_width_m, "메인로버": main_rover.track_width_m},
        "무게중심 높이 m": {"정찰로버": scout_rover.cg_height_m, "메인로버": main_rover.cg_height_m},
        "지상고 m": {"정찰로버": scout_rover.ground_clearance_m, "메인로버": main_rover.ground_clearance_m},
        "최대 토크 Nm": {"정찰로버": scout_rover.max_wheel_torque_nm, "메인로버": main_rover.max_wheel_torque_nm},
    }
    return pd.DataFrame.from_dict(rows, orient="index")

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


def launch_irrlicht_rover_viewer(
    rover_key: str,
    condition_id: str,
    duration_s: float = 6.0,
    torque_fraction: float = 0.6,
) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "src.chrono.irrlicht_rover_viewer",
        "--rover",
        rover_key,
        "--condition",
        condition_id,
        "--duration",
        f"{duration_s:.3f}",
        "--torque-fraction",
        f"{torque_fraction:.3f}",
    ]
    creationflags = subprocess.CREATE_NEW_CONSOLE if hasattr(subprocess, "CREATE_NEW_CONSOLE") else 0
    launch_command = ["cmd", "/k", *command] if sys.platform.startswith("win") else command
    subprocess.Popen(launch_command, cwd=APP_DIR, creationflags=creationflags)
    return command

def render_integration_usage_guide() -> None:
    with st.expander("이 탭 사용법", expanded=True):
        st.markdown(
            """
1. 먼저 `RoverSpec`, `TerrainScenario`, `ControlProfile`을 고릅니다. 이 세 개가 실험의 기본 입력입니다.
2. `ScoutObservation`은 정찰로버가 실제로 측정한 데이터입니다. 맞는 파일이 없으면 비워둘 수 있습니다.
3. `ContactPairSpec`은 바퀴-지형 접촉값입니다. 아직 측정값이 없으면 비워두고 임시값으로 봅니다.
4. `실험 실행`은 위험도 계산용입니다. `heuristic`은 기존 MVP 계산식, `mock_chrono`는 아직 물리 시뮬레이션이 아닌 연결 확인용입니다.
5. `PyChrono 설치 확인 실행`은 설치 확인용입니다. 1 kg 박스를 떨어뜨리는 테스트라서 로버 위험도로 해석하면 안 됩니다.
6. `박스 낙하 Viewer 열기`는 박스 낙하를 창으로 보는 버튼입니다.
7. `계산 결과를 Irrlicht로 보기`는 `실험 실행` 후 결과 아래에서 정찰로버 주행과 메인로버 주행을 순서대로 확인하는 버튼입니다.

Smoke가 정상이라면 `status=completed`, `contact_detected=True`, `max_contact_count >= 1`, `final_z < 0.25`가 나옵니다.
            """.strip()
        )



def numeric_items(data: dict) -> dict[str, float]:
    out: dict[str, float] = {}
    for key, value in data.items():
        if isinstance(value, bool):
            out[key] = 1.0 if value else 0.0
        elif isinstance(value, (int, float)) and value is not None:
            out[key] = float(value)
    return out


def plot_selected_terrain_schematic(terrain) -> plt.Figure:
    length_x, width_y, _height_z = terrain.dimensions_xyz_m
    fig, ax = plt.subplots(figsize=(6, 3.5))
    floor = patches.Rectangle(
        (-length_x / 2.0, -width_y / 2.0),
        length_x,
        width_y,
        facecolor="#eef2f5",
        edgecolor="#2f3a45",
        linewidth=1.5,
    )
    ax.add_patch(floor)
    ax.plot([-length_x / 2.0, length_x / 2.0], [0, 0], color="#4b5563", linewidth=1.0, alpha=0.6)
    for obstacle in terrain.obstacles:
        ox, oy, _oz = obstacle.pose.xyz_m
        sx, sy, _sz = obstacle.dimensions_xyz_m
        rect = patches.Rectangle(
            (ox - sx / 2.0, oy - sy / 2.0),
            sx,
            sy,
            facecolor="#c2410c",
            edgecolor="#7c2d12",
            alpha=0.75,
        )
        ax.add_patch(rect)
        ax.text(ox, oy, obstacle.obstacle_id, ha="center", va="center", fontsize=8, color="white")
    ax.set_title(f"{terrain.terrain_id} 평면도")
    ax.set_xlabel("X 전진 방향 (m)")
    ax.set_ylabel("Y 좌우 방향 (m)")
    ax.set_xlim(-length_x / 2.0 - 0.1, length_x / 2.0 + 0.1)
    ax.set_ylim(-width_y / 2.0 - 0.1, width_y / 2.0 + 0.1)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)
    return fig


def infer_rigid_viewer_selection(rover, terrain) -> tuple[str | None, str | None, str]:
    rover_id = rover.rover_id.lower()
    if "main" in rover_id:
        rover_key = "main"
    elif "scout" in rover_id:
        rover_key = "scout"
    else:
        return None, None, "현재 Irrlicht 로버 viewer는 scout_v01/main_v01 계열만 바로 실행할 수 있습니다."

    if terrain.obstacles:
        max_height = max(float(obs.dimensions_xyz_m[2]) for obs in terrain.obstacles)
        condition_id = "obstacle_high" if max_height >= 0.04 else "obstacle_low"
        return rover_key, condition_id, "선택한 obstacle 시나리오를 rigid pilot obstacle preset으로 변환해 보여줍니다."

    slope = abs(float(terrain.slope_long_deg))
    if slope >= 12.0:
        condition_id = "slope_15deg"
    elif slope >= 7.5:
        condition_id = "slope_10deg"
    elif slope >= 3.0:
        condition_id = "slope_5deg"
    else:
        condition_id = "flat"
    note = "현재 viewer는 선택한 Contract 시나리오를 가장 가까운 rigid pilot preset으로 변환합니다."
    if condition_id.startswith("slope_"):
        note += " slope preset은 아직 실제 경사 지형이 아니라 flat 바닥+기울어진 중력 모델입니다."
    return rover_key, condition_id, note


def render_selected_experiment_visualization(rover, scout_rover, terrain, control) -> None:
    st.subheader("선택한 실험 한눈에 보기")
    st.caption("계산 전에는 입력값만 확인합니다. Irrlicht 시각화는 `실험 실행` 후 결과 아래에서 실행합니다.")

    rover_compare = rover_parameter_comparison_frame(rover, scout_rover)
    terrain_values = {
        "길이 X m": terrain.dimensions_xyz_m[0],
        "폭 Y m": terrain.dimensions_xyz_m[1],
        "높이/두께 Z m": terrain.dimensions_xyz_m[2],
        "종방향 경사 deg": terrain.slope_long_deg,
        "횡방향 경사 deg": terrain.slope_lat_deg,
        "거칠기 m": terrain.roughness_m,
        "틈 폭 m": terrain.gap_width_m,
        "장애물 수": len(terrain.obstacles),
    }
    control_values = {
        "목표 속도 m/s": control.target_speed_mps,
        "실행 시간 s": control.duration_s,
        "throttle": control.throttle,
        "조향각 deg": control.steering_deg,
    }

    tabs = st.tabs(["정찰/메인 로버 비교", "지형", "제어"])
    with tabs[0]:
        st.caption("정찰로버는 ScoutObservation을 만든 주체이고, 메인로버는 위험도를 예측할 대상입니다.")
        st.bar_chart(rover_compare)
        st.dataframe(rover_compare, use_container_width=True)
    with tabs[1]:
        st.warning("종민님이 전달한 arena/map.py 전체 환경은 아직 이 Contract 선택값에서 직접 생성되지 않습니다. 현재는 YAML 요약도와 rigid pilot preset 변환으로 확인하며, 다음 단계에서 TerrainFactory가 Jongmin arena를 호출하도록 연결해야 합니다.")
        left, right = st.columns([1, 1])
        with left:
            st.pyplot(plot_selected_terrain_schematic(terrain))
        with right:
            st.bar_chart(pd.Series(terrain_values, name="값"))
            st.dataframe(pd.Series(terrain_values, name="값").to_frame(), use_container_width=True)
    with tabs[2]:
        st.bar_chart(pd.Series(control_values, name="값"))
        st.dataframe(pd.Series(control_values, name="값").to_frame(), use_container_width=True)

def render_pychrono_environment() -> None:
    availability = get_pychrono_availability()
    st.subheader("PyChrono 환경 확인")
    cols = st.columns(5)
    cols[0].metric("PyChrono", "사용 가능" if availability.pychrono_available else "사용 불가")
    cols[1].metric("Vehicle 모듈", "사용 가능" if availability.vehicle_module_available else "사용 불가")
    cols[2].metric("Irrlicht 모듈", "사용 가능" if availability.irrlicht_module_available else "사용 불가")
    cols[3].metric("버전", availability.version or "unknown")
    cols[4].metric("Python", Path(availability.python_executable).name)
    st.caption(f"Python 실행 파일: {availability.python_executable}")
    st.caption(f"모듈 경로: {availability.pychrono_module_path or '-'}")
    if availability.pychrono_available:
        st.success(availability.diagnostic_message)
    else:
        st.warning(availability.diagnostic_message)

    with st.expander("박스 낙하 smoke viewer", expanded=False):
        st.write(
            "이 버튼은 PyChrono/Irrlicht 설치 확인용입니다. 로버 모델과는 관계없는 1 kg 박스 낙하만 보여줍니다. "
            "로버 시각화는 `실험 실행` 후 결과 아래의 `계산 결과 시각화`에서 실행하세요."
        )
        viewer_duration = st.number_input("박스 낙하 viewer 실행 시간 (초)", min_value=1.0, max_value=60.0, value=10.0, step=1.0)
        disabled = not availability.pychrono_available or not availability.irrlicht_module_available
        if st.button("박스 낙하 Viewer 열기", disabled=disabled):
            try:
                command = launch_irrlicht_smoke_viewer(viewer_duration)
                st.success("박스 낙하 viewer를 별도 창으로 열었습니다.")
                st.code(" ".join(command))
            except Exception as exc:
                st.error(f"박스 낙하 viewer 실행 실패: {type(exc).__name__}: {exc}")
        st.code('conda activate chrono\ncd /d "C:\\K_SRC\\mobility_twin_mvp"\npython -m src.chrono.irrlicht_smoke_viewer --duration 10')


def render_simulation_result(result: SimulationResult, path: str | None = None) -> None:
    if result.backend_name == "mock_chrono":
        st.error(
            "MOCK CHRONO: 실제 Chrono 물리엔진을 실행한 결과가 아닙니다. "
            "연결 구조 확인용이며 로버 위험도 검증값으로 쓰면 안 됩니다."
        )
    if result.backend_name == "pychrono_smoke":
        st.info("PyChrono Smoke는 1 kg 박스 낙하 환경 확인입니다. 로버 이동 위험도 계산이 아닙니다.")

    st.subheader("SimulationResult 결과")
    metric_cols = st.columns(5)
    metric_cols[0].metric("계산 방식", result.backend_name)
    metric_cols[1].metric("모델 상태", result.model_status)
    metric_cols[2].metric("평가 상태", result.evaluation_state)
    metric_cols[3].metric("등급", result.grade)
    metric_cols[4].metric("위험도", "N/A" if result.final_risk is None else f"{result.final_risk:.2f}")

    if path:
        st.caption(f"저장된 JSON: {path}")

    risk_numeric = numeric_items(result.risk_components)
    typed_numeric = numeric_items(result.metrics_typed.to_dict())
    metrics_numeric = numeric_items(result.metrics)

    tabs = st.tabs(["위험도 그래프", "물리 지표", "궤적", "JSON"])
    with tabs[0]:
        if risk_numeric:
            st.bar_chart(pd.Series(risk_numeric, name="risk"))
            st.dataframe(pd.Series(risk_numeric, name="risk").to_frame(), use_container_width=True)
        else:
            st.info("이 결과에는 위험도 구성요소가 없습니다. smoke/mock 결과는 보통 위험도 평가에 포함되지 않습니다.")
    with tabs[1]:
        if typed_numeric:
            st.bar_chart(pd.Series(typed_numeric, name="value"))
            st.dataframe(pd.Series(typed_numeric, name="value").to_frame(), use_container_width=True)
        if metrics_numeric:
            st.write("추가 metrics")
            st.bar_chart(pd.Series(metrics_numeric, name="value"))
            st.dataframe(pd.Series(metrics_numeric, name="value").to_frame(), use_container_width=True)
    with tabs[2]:
        trajectory_csv = result.artifacts.get("trajectory_csv") if result.artifacts else None
        if trajectory_csv and Path(trajectory_csv).exists():
            trajectory = pd.read_csv(trajectory_csv)
            st.dataframe(trajectory, use_container_width=True)
            time_col = "time_s" if "time_s" in trajectory.columns else "t_s" if "t_s" in trajectory.columns else None
            z_col = "position_z_m" if "position_z_m" in trajectory.columns else "z_m" if "z_m" in trajectory.columns else None
            x_col = "position_x_m" if "position_x_m" in trajectory.columns else "x_m" if "x_m" in trajectory.columns else None
            y_col = "position_y_m" if "position_y_m" in trajectory.columns else "y_m" if "y_m" in trajectory.columns else None
            speed_col = "velocity_x_mps" if "velocity_x_mps" in trajectory.columns else "v_forward_mps" if "v_forward_mps" in trajectory.columns else None
            if time_col and z_col:
                st.write("높이 z 변화")
                st.line_chart(trajectory.set_index(time_col)[z_col])
            if time_col and speed_col:
                st.write("속도 변화")
                st.line_chart(trajectory.set_index(time_col)[speed_col])
            if x_col and y_col:
                st.write("XY 이동 경로")
                st.scatter_chart(trajectory[[x_col, y_col]].rename(columns={x_col: "x", y_col: "y"}))
            if time_col and "contact_count" in trajectory.columns:
                st.write("접촉 수 변화")
                st.line_chart(trajectory.set_index(time_col)["contact_count"])
        else:
            st.info("이 결과에는 trajectory CSV가 없습니다. heuristic 결과는 수식 기반이라 궤적이 생성되지 않습니다.")
    with tabs[3]:
        if result.artifacts:
            st.write("생성 파일")
            st.json(result.artifacts)
        st.write("전체 JSON")
        st.json(result.to_dict())


def sidebar_configs() -> tuple[MainRoverConfig, ScoutReferenceConfig]:
    rover_ids = ROVER_REGISTRY.ids()

    st.header("메인로버 설정")
    main_mode = st.radio("메인로버 입력 방식", ["모델 선택", "직접 입력"], horizontal=True)
    main_editable = main_mode == "직접 입력"
    main_model_id = st.selectbox("메인로버 모델", rover_ids, index=rover_ids.index("main_v01") if "main_v01" in rover_ids else 0)
    main_defaults = main_config_from_rover_spec(ROVER_REGISTRY.load(main_model_id)) if main_mode == "모델 선택" else DEFAULT_MAIN_ROVER
    if main_mode == "모델 선택":
        selected = ROVER_REGISTRY.load(main_model_id)
        st.caption(f"모델값 사용: {selected.display_name}")
        if selected.fallback_mu_eff is None or selected.fallback_crr is None:
            st.warning("이 로버 YAML에는 휴리스틱용 마찰/구름저항 값이 없어 MVP 기본값을 임시 사용합니다.")

    main = MainRoverConfig(
        mass_kg=st.number_input("메인 질량 (kg)", min_value=1.0, value=main_defaults.mass_kg, step=1.0, disabled=not main_editable),
        wheel_radius_m=st.number_input("메인 바퀴 반지름 (m)", min_value=0.01, value=main_defaults.wheel_radius_m, step=0.01, disabled=not main_editable),
        wheel_width_m=st.number_input("메인 바퀴 폭 (m)", min_value=0.01, value=main_defaults.wheel_width_m, step=0.01, disabled=not main_editable),
        wheelbase_m=st.number_input("메인 휠베이스 (m)", min_value=0.05, value=main_defaults.wheelbase_m, step=0.05, disabled=not main_editable),
        track_width_m=st.number_input("메인 좌우 바퀴 간격 (m)", min_value=0.05, value=main_defaults.track_width_m, step=0.05, disabled=not main_editable),
        cg_height_m=st.number_input("메인 무게중심 높이 (m)", min_value=0.02, value=main_defaults.cg_height_m, step=0.01, disabled=not main_editable),
        ground_clearance_m=st.number_input("메인 지상고 (m)", min_value=0.01, value=main_defaults.ground_clearance_m, step=0.01, disabled=not main_editable),
        driven_wheel_count=st.number_input("메인 구동 바퀴 수", min_value=1, max_value=12, value=main_defaults.driven_wheel_count, step=1, disabled=not main_editable),
        max_wheel_torque_nm=st.number_input("메인 바퀴 최대 토크 (Nm)", min_value=0.1, value=main_defaults.max_wheel_torque_nm, step=1.0, disabled=not main_editable),
        mu_eff=st.number_input("유효 마찰계수 mu", min_value=0.01, max_value=2.0, value=main_defaults.mu_eff, step=0.05, disabled=not main_editable),
        crr=st.number_input("구름저항 Crr", min_value=0.0, max_value=1.0, value=main_defaults.crr, step=0.01, disabled=not main_editable),
    )

    st.header("정찰로버 기준값")
    scout_mode = st.radio("정찰로버 입력 방식", ["모델 선택", "직접 입력"], horizontal=True)
    scout_editable = scout_mode == "직접 입력"
    scout_model_id = st.selectbox("정찰로버 모델", rover_ids, index=rover_ids.index("scout_v01") if "scout_v01" in rover_ids else 0)
    scout_defaults = scout_reference_from_rover_spec(ROVER_REGISTRY.load(scout_model_id)) if scout_mode == "모델 선택" else DEFAULT_SCOUT_REFERENCE
    if scout_mode == "모델 선택":
        selected = ROVER_REGISTRY.load(scout_model_id)
        st.caption(f"모델값 사용: {selected.display_name}")

    scout = ScoutReferenceConfig(
        mass_kg=st.number_input("정찰 질량 (kg)", min_value=0.1, value=scout_defaults.mass_kg, step=0.5, disabled=not scout_editable),
        wheel_radius_m=st.number_input("정찰 바퀴 반지름 (m)", min_value=0.005, value=scout_defaults.wheel_radius_m, step=0.005, disabled=not scout_editable),
        wheel_width_m=st.number_input("정찰 바퀴 폭 (m)", min_value=0.005, value=scout_defaults.wheel_width_m, step=0.005, disabled=not scout_editable),
        wheel_load_n=st.number_input("정찰 바퀴당 하중 (N)", min_value=0.1, value=scout_defaults.wheel_load_n, step=1.0, disabled=not scout_editable),
    )

    if st.button("샘플 CSV 다시 만들기"):
        save_sample_csv(SAMPLE_CSV)
        st.success("sample_patches.csv를 다시 만들었습니다.")

    return main, scout

def render_scout_to_main_prediction_preview(rover, scout_rover, terrain, control, observation, contact_pair) -> None:
    st.subheader("정찰로버 데이터로 예측한 메인로버 위험도")
    st.caption("이 부분이 MVP의 핵심입니다. 선택한 정찰로버 모델의 하중/바퀴 파라미터와 ScoutObservation 관측값을 메인로버 물리값에 맞춰 스케일링합니다.")

    resolved_observation = observation or terrain.legacy_observation(rover.rover_id, control.profile_id)
    if resolved_observation is None:
        st.warning("이 조합에는 정찰로버 관측 데이터가 없습니다. ScoutObservation을 선택하거나 TerrainScenario에 legacy scout_response가 있어야 예측할 수 있습니다.")
        return

    if resolved_observation.scout_rover_id != scout_rover.rover_id:
        st.warning(f"관측 파일의 scout_rover_id는 `{resolved_observation.scout_rover_id}`이고, 현재 선택한 정찰로버 모델은 `{scout_rover.rover_id}`입니다. 현재 MVP에서는 선택한 정찰로버 모델 파라미터로 스케일링합니다.")

    if contact_pair is None and (rover.fallback_mu_eff is None or rover.fallback_crr is None):
        st.warning("이 로버에는 fallback 마찰/구름저항 값이 없고 ContactPairSpec도 선택되지 않았습니다. 접촉값을 선택해야 예측할 수 있습니다.")
        return

    scout_reference = scout_reference_from_rover_spec(scout_rover)
    result = HeuristicBackend(scout_reference=scout_reference).run(rover, terrain, control, observation=observation, contact_pair=contact_pair)
    if result.evaluation_state != "EVALUATED":
        st.warning("현재 선택값으로는 예측이 평가되지 않았습니다.")
        st.json(result.to_dict())
        return

    metrics = result.metrics
    scout_inputs = {
        "정찰 slip": metrics.get("observation_mean_slip"),
        "정찰 sinkage m": metrics.get("observation_mean_sinkage_m"),
        "정찰 wheel torque Nm": resolved_observation.mean_wheel_torque_nm,
        "정찰 COT": resolved_observation.cot,
        "진동 RMS g": resolved_observation.vibration_rms_g,
        "종방향 경사 deg": resolved_observation.slope_long_deg,
        "횡방향 경사 deg": resolved_observation.slope_lat_deg,
        "장애물 높이 m": resolved_observation.obstacle_height_m,
        "gap 폭 m": resolved_observation.gap_width_m,
    }
    predicted_main = {
        "예측 메인 slip": metrics.get("physics_predicted_main_slip"),
        "예측 메인 sinkage m": metrics.get("physics_predicted_main_sinkage_m"),
        "요구 견인력 F_req N": metrics.get("physics_f_req_n"),
        "토크 한계 힘 F_torque N": metrics.get("physics_f_torque_n"),
        "마찰 한계 힘 F_friction N": metrics.get("physics_f_friction_n"),
        "사용 가능 힘 F_avail N": metrics.get("physics_f_avail_n"),
        "견인 여유 N": metrics.get("physics_traction_margin_n"),
        "전복 여유 deg": metrics.get("physics_tipover_margin_deg"),
    }
    prediction_summary = {
        "최종 위험도": result.final_risk,
        "예측 신뢰도": result.prediction_confidence,
        "contact mu_eff": metrics.get("contact_mu_eff"),
        "contact crr_eff": metrics.get("contact_crr_eff"),
        "선택 정찰 wheel load N": scout_reference.wheel_load_n,
    }

    st.markdown(
        """
예측에 쓰는 변수와 계산 흐름:
1. 정찰 관측값: `scout_slip`, `scout_sinkage_m`, `scout_wheel_torque_nm`, `scout_cot`, `vibration_rms_g`, 경사/장애물/gap.
2. 메인로버 물리값: 질량, 바퀴 반지름, 바퀴 폭, 휠베이스, track width, 무게중심 높이, 지상고, 최대 바퀴 토크.
3. 접촉값: `mu_eff`, `crr_eff`.
4. 메인로버 예측: 정찰 slip/sinkage를 접지압 비율과 바퀴 크기 비율로 스케일링합니다.
5. 위험도 계산: 견인력, 전복, 장애물, gap, slip, sinkage, 에너지, 진동, 불확실성 risk를 0~1로 만들고 가중합합니다.
        """.strip()
    )

    cols = st.columns(4)
    cols[0].metric("최종 위험도", "N/A" if result.final_risk is None else f"{result.final_risk:.2f}")
    cols[1].metric("등급", result.grade)
    cols[2].metric("예측 메인 slip", f"{metrics.get('physics_predicted_main_slip', 0.0):.3f}")
    cols[3].metric("예측 메인 sinkage", f"{metrics.get('physics_predicted_main_sinkage_m', 0.0):.3f} m")

    if result.hard_failure_reasons:
        st.error("Hard failure: " + ", ".join(result.hard_failure_reasons))

    tabs = st.tabs(["정찰 입력", "메인 예측값", "위험도 구성", "계산식 메모"])
    with tabs[0]:
        scout_series = pd.Series({k: v for k, v in scout_inputs.items() if v is not None}, name="값")
        st.bar_chart(scout_series)
        st.dataframe(scout_series.to_frame(), use_container_width=True)
    with tabs[1]:
        main_series = pd.Series({k: v for k, v in predicted_main.items() if v is not None}, name="값")
        st.bar_chart(main_series)
        st.dataframe(main_series.to_frame(), use_container_width=True)
        st.write("요약")
        summary_series = pd.Series({k: v for k, v in prediction_summary.items() if v is not None}, name="값")
        st.dataframe(summary_series.to_frame(), use_container_width=True)
    with tabs[2]:
        risk_series = pd.Series(result.risk_components, name="risk")
        st.bar_chart(risk_series)
        st.dataframe(risk_series.to_frame(), use_container_width=True)
    with tabs[3]:
        st.code(
            """
F_req = m*g*sin(abs(slope_long)) + Crr*m*g*cos(abs(slope_long))
F_torque = driven_wheel_count * max_wheel_torque / wheel_radius
F_friction = mu_eff * m*g*cos(abs(slope_long))
F_avail = min(F_torque, F_friction)
traction_margin = F_avail - F_req
pressure_ratio = (main_mass*g / driven_wheel_count) / scout_wheel_load_N
predicted_main_sinkage = scout_sinkage * sqrt(pressure_ratio) * sqrt(scout_wheel_width / main_wheel_width)
predicted_main_slip = clip(scout_slip * sqrt(pressure_ratio) * sqrt(scout_wheel_radius / main_wheel_radius), 0, 1)
final_risk = weighted_sum(traction, tipover, obstacle, gap, slip, sinkage, energy, vibration, uncertainty)
            """.strip()
        )

def render_post_run_irrlicht_viewer(result: SimulationResult, rover, scout_rover, terrain, control) -> None:
    st.subheader("계산 결과 시각화")
    st.caption("실험 실행 결과를 확인한 뒤, 같은 조건을 정찰로버가 먼저 주행하고 이어서 메인로버가 주행하는 순서로 봅니다.")
    if result.backend_name == "pychrono_smoke":
        st.info("현재 결과는 박스 낙하 smoke 결과라 로버 viewer와 연결하지 않습니다.")
        return

    availability = get_pychrono_availability()
    _rover_key, condition_id, note = infer_rigid_viewer_selection(rover, terrain)
    st.write(note)
    if condition_id:
        st.info(f"viewer preset: scout_then_main, condition={condition_id}")
    disabled = not (availability.pychrono_available and availability.irrlicht_module_available and condition_id)
    viewer_duration = st.number_input(
        "시각화 실행 시간 (초)", min_value=1.0, max_value=30.0, value=float(min(control.duration_s, 10.0)), step=1.0, key="post_viewer_duration"
    )
    torque_fraction = st.slider(
        "시각화 토크 비율", min_value=0.0, max_value=1.0, value=max(0.05, min(float(control.throttle), 1.0)), step=0.05, key="post_viewer_torque"
    )
    if st.button("계산 결과를 Irrlicht로 보기", disabled=disabled):
        try:
            command = launch_irrlicht_rover_viewer("scout_then_main", condition_id, viewer_duration, torque_fraction)
            st.success("정찰로버 주행 후 메인로버 주행 viewer를 별도 창으로 열었습니다. 첫 창을 닫으면 다음 로버 주행이 시작됩니다.")
            st.code(" ".join(command))
        except Exception as exc:
            st.error(f"viewer 실행 실패: {type(exc).__name__}: {exc}")
    if disabled:
        st.warning("PyChrono/Irrlicht가 없거나, 선택한 로버/지형을 현재 viewer preset으로 변환할 수 없습니다. Streamlit을 chrono 환경에서 실행했는지 확인하세요.")

def render_integration_experiment() -> None:
    st.caption("통합 실험: 로버 모델 + 지형 시나리오 + 정찰 관측 + 접촉 조건 + 제어 프로파일을 SimulationResult로 변환합니다.")
    render_integration_usage_guide()
    render_pychrono_environment()

    rover_ids = ROVER_REGISTRY.ids()
    terrain_ids = TERRAIN_REGISTRY.ids()
    control_ids = CONTROL_REGISTRY.ids()
    if not rover_ids or not terrain_ids or not control_ids:
        st.warning("YAML 파일을 찾을 수 없습니다. rover_models, terrain_scenarios, control_profiles 폴더를 확인하세요.")
        return

    col1, col2, col3, col4 = st.columns(4)
    target_rover_ids = [item for item in rover_ids if "main" in item] or rover_ids
    scout_rover_ids = [item for item in rover_ids if "scout" in item] or rover_ids
    rover_id = col1.selectbox("예측 대상 메인로버", target_rover_ids, index=target_rover_ids.index("main_v01") if "main_v01" in target_rover_ids else 0)
    scout_rover_id = col2.selectbox("정찰로버 모델", scout_rover_ids, index=scout_rover_ids.index("scout_v01") if "scout_v01" in scout_rover_ids else 0)
    terrain_id = col3.selectbox("지형 시나리오", terrain_ids)
    control_id = col4.selectbox("제어 프로파일", control_ids)

    rover = ROVER_REGISTRY.load(rover_id)
    scout_rover = ROVER_REGISTRY.load(scout_rover_id)
    terrain = TERRAIN_REGISTRY.load(terrain_id)
    control = CONTROL_REGISTRY.load(control_id)
    st.session_state["selected_rover_id"] = rover_id
    st.caption("메인로버는 위험도를 예측할 대상이고, 정찰로버는 ScoutObservation을 만든 로버입니다. 아래 예측식에는 선택한 정찰로버의 바퀴 하중/반지름/폭이 들어갑니다.")
    observation_matches = OBSERVATION_REGISTRY.ids_for(terrain_id=terrain_id, control_profile_id=control_id)
    observation_options = ["<없음: 가능한 경우 기존값 사용>"] + observation_matches
    contact_matches = CONTACT_PAIR_REGISTRY.ids_for(rover.wheel_material_id, terrain.material_id)
    contact_options = ["<없음: 로버 기본값 사용>"] + contact_matches

    col4, col5, col6 = st.columns(3)
    observation_id = col4.selectbox("정찰 관측 데이터", observation_options, index=1 if observation_matches else 0)
    contact_pair_id = col5.selectbox("바퀴-지형 접촉값", contact_options, index=1 if contact_matches else 0)
    backend_id = col6.selectbox("계산 방식", ["heuristic", "mock_chrono"])

    observation = None if observation_id.startswith("<") else OBSERVATION_REGISTRY.load(observation_id)
    contact_pair = None if contact_pair_id.startswith("<") else CONTACT_PAIR_REGISTRY.load(contact_pair_id)
    if contact_pair is not None:
        CONTACT_PAIR_REGISTRY.validate_references(contact_pair, rover, terrain, TERRAIN_MATERIAL_REGISTRY)

    material = TERRAIN_MATERIAL_REGISTRY.load(terrain.material_id) if terrain.material_id in TERRAIN_MATERIAL_REGISTRY.ids() else None
    if material and material.parameter_source == "assumed":
        st.info(f"Terrain material `{material.material_id}` uses assumed parameters, confidence={material.confidence:.2f}.")
    if contact_pair and contact_pair.source == "assumed":
        st.info(f"Contact pair `{contact_pair.contact_pair_id}` uses assumed parameters, confidence={contact_pair.confidence:.2f}.")

    render_selected_experiment_visualization(rover, scout_rover, terrain, control)
    render_scout_to_main_prediction_preview(rover, scout_rover, terrain, control, observation, contact_pair)

    with st.expander("선택한 handoff schema 원본", expanded=False):
        tabs = st.tabs(["Main RoverSpec", "Scout RoverSpec", "TerrainScenario", "ScoutObservation", "ContactPairSpec", "ControlProfile"])
        tabs[0].json(rover.to_dict())
        tabs[1].json(scout_rover.to_dict())
        tabs[2].json(terrain.to_dict())
        tabs[3].json({} if observation is None else observation.to_dict())
        tabs[4].json({} if contact_pair is None else contact_pair.to_dict())
        tabs[5].json(control.to_dict())

    col_run_1, col_run_2 = st.columns(2)
    if col_run_1.button("실험 실행", type="primary"):
        backend = HeuristicBackend(scout_reference=scout_reference_from_rover_spec(scout_rover)) if backend_id == "heuristic" else make_backend(backend_id)
        result = backend.run(rover, terrain, control, observation=observation, contact_pair=contact_pair)
        output_path = save_simulation_result(result)
        st.session_state["last_simulation_result"] = result
        st.session_state["last_simulation_result_path"] = str(output_path)

    if col_run_2.button("PyChrono 설치 확인 실행"):
        with st.status("실행 중", expanded=True) as status:
            backend = make_backend("pychrono_smoke")
            result = backend.run(rover, terrain, control, observation=observation, contact_pair=contact_pair)
            output_path = save_simulation_result(result)
            st.session_state["last_simulation_result"] = result
            st.session_state["last_simulation_result_path"] = str(output_path)
            if result.status == "completed":
                status.update(label="완료", state="complete")
            elif result.status == "timeout":
                status.update(label="시간 초과", state="error")
            else:
                status.update(label="실패", state="error")

    result = st.session_state.get("last_simulation_result")
    if result:
        render_simulation_result(result, st.session_state.get("last_simulation_result_path"))
        render_post_run_irrlicht_viewer(result, rover, scout_rover, terrain, control)


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
    st.info("이 화면은 기존 MVP 휴리스틱 지도입니다. Chrono 물리 결과가 아니라 정찰 데이터와 로버 물리값으로 위험도를 빠르게 계산하는 검증용 화면입니다.")
    st.markdown(
        """
사용 순서:
1. 아래 설정 패널에서 메인로버와 정찰로버를 `모델 선택` 또는 `직접 입력`으로 정합니다.
2. 가운데 표에서 지형/정찰 측정값을 확인하거나 수정합니다.
3. `패치 위험도 계산`을 누르면 메인로버 기준 위험도 지도가 갱신됩니다.
        """.strip()
    )

    main_config, scout_reference = sidebar_configs()
    source = pd.read_csv(SAMPLE_CSV)
    edited = st.data_editor(source, use_container_width=True, num_rows="dynamic")

    if st.button("패치 위험도 계산", type="primary") or "last_results" not in st.session_state:
        st.session_state["last_results"] = analyze_dataframe(edited, main_config, scout_reference)

    results = st.session_state["last_results"]
    st.subheader("패치별 위험도 표")
    st.dataframe(
        results[["patch_id", "grid_x", "grid_y", "terrain_label", "risk_score", "grade", "hard_failure_reasons"]],
        use_container_width=True,
        hide_index=True,
    )

    render_summary(results)

    st.subheader("메인로버 이동 위험도 지도")
    st.pyplot(plot_traversability_map(results))

    render_detail(results)


def main() -> None:
    st.set_page_config(page_title="정찰로버-메인로버 Mobility Twin MVP", layout="wide")
    ensure_sample_csv()

    st.title("정찰로버-메인로버 Mobility Twin MVP")
    st.write(
        "정찰로버 관측값을 메인로버 기준 이동 가능성/위험도 추정으로 변환하는 MVP입니다. "
        "Chrono 연결부와 기존 휴리스틱 지도를 함께 확인할 수 있습니다."
    )
    st.caption("최종 목표: 메인로버 기준 이동 위험도 지도")

    tab_integration, tab_legacy = st.tabs(["통합 실험", "기존 휴리스틱 위험도 지도"])
    with tab_integration:
        render_integration_experiment()
    with tab_legacy:
        render_legacy_risk_map()


if __name__ == "__main__":
    main()
