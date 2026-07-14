"""빌더 구성/질량/명령 테스트 + 스폰 sanity (headless, 짧은 시뮬레이션)."""

from pathlib import Path

import pytest

import pychrono as chrono

from rover_builder import (
    WHEEL_NAMES,
    apply_collision_defaults,
    build_rover,
    overall_dimensions,
)
from rover_schema import SimConfig, load_spec
from test_ground import make_test_ground, spawn_frame

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SPEC_DIR = PROJECT_ROOT / "specs" / "rovers"


def fresh_system(cfg: SimConfig) -> chrono.ChSystemNSC:
    apply_collision_defaults(cfg)
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -cfg.gravity_mps2))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    return system


@pytest.mark.parametrize("yaml_name", ["scout_v01.yaml", "main_v01.yaml"])
def test_total_mass_matches_spec(yaml_name):
    spec, cfg = load_spec(SPEC_DIR / yaml_name)
    system = fresh_system(cfg)
    rover = build_rover(system, spec, cfg)
    assert abs(rover.total_mass() - spec.mass_kg) < 1e-6


def test_structure_4wd():
    spec, cfg = load_spec(SPEC_DIR / "scout_v01.yaml")
    system = fresh_system(cfg)
    rover = build_rover(system, spec, cfg)
    assert set(rover.wheels) == set(WHEEL_NAMES)
    assert set(rover.motors) == set(WHEEL_NAMES)   # 4WD → 전휠 모터
    assert not rover.free_joints
    # 시스템 소유권: 빌더가 만든 바디가 외부 system 에 들어갔는지
    assert system.GetBodies() and len(system.GetBodies()) == 5  # 섀시+휠4


def test_wheel_positions_follow_spec():
    spec, cfg = load_spec(SPEC_DIR / "scout_v01.yaml")
    system = fresh_system(cfg)
    rover = build_rover(system, spec, cfg)
    fl = rover.wheels["FL"].GetPos()
    rr = rover.wheels["RR"].GetPos()
    assert abs((fl.x - rr.x) - spec.wheelbase_m) < 1e-9
    assert abs((fl.y - rr.y) - spec.track_width_m) < 1e-9
    assert abs(fl.z - (spec.wheel_radius_m + cfg.spawn_drop_m)) < 1e-9


def test_speed_command_clamped():
    spec, cfg = load_spec(SPEC_DIR / "scout_v01.yaml")
    system = fresh_system(cfg)
    rover = build_rover(system, spec, cfg)
    rover.set_command(10.0 * spec.max_wheel_speed_radps)
    assert rover.commanded_lr() == pytest.approx(
        (spec.max_wheel_speed_radps,) * 2)
    rover.set_command(-10.0 * spec.max_wheel_speed_radps)
    assert rover.commanded_lr() == pytest.approx(
        (-spec.max_wheel_speed_radps,) * 2)


def test_scout_fits_stowage_volume():
    """공고문 수납부피 300×300×200 mm (마스트 접이 제외) — 대회 출품 scout."""
    spec, cfg = load_spec(SPEC_DIR / "scout_v01.yaml")
    L, W, H = overall_dimensions(spec, cfg)
    assert L <= 0.30 and W <= 0.30 and H <= 0.20, (L, W, H)


def test_pivot_turn_changes_heading():
    """좌우 역회전 명령(skid steer) → 제자리 좌선회 (yaw +)."""
    spec, cfg = load_spec(SPEC_DIR / "scout_v01.yaml")
    system = fresh_system(cfg)
    make_test_ground(system, cfg)
    rover = build_rover(system, spec, cfg, spawn_frame(0.0, 0.0))

    v = 0.3 * spec.max_wheel_speed_radps
    rover.set_command_lr(-v, v)
    assert rover.commanded_lr() == pytest.approx((-v, v))

    while system.GetChTime() < 2.0:
        system.DoStepDynamics(cfg.timestep_s)
        rover.update(cfg.timestep_s)

    # yaw 각은 ±180° 래핑되므로 각속도 부호로 방향을 판정한다
    yaw_rate = float(rover.chassis.GetAngVelLocal().z)
    assert yaw_rate > 0.2, f"좌선회 명령인데 yaw rate = {yaw_rate:.3f} rad/s"
    pos = rover.chassis.GetPos()
    assert abs(pos.x) < 0.1 and abs(pos.y) < 0.1, "제자리 선회인데 위치 이탈"


def test_wheel_torque_mode_drives_forward():
    """command_type=wheel_torque 분기: 토크 명령으로 전진 + 선형 맵 클램프."""
    import dataclasses
    spec, cfg = load_spec(SPEC_DIR / "scout_v01.yaml")
    spec = dataclasses.replace(spec, command_type="wheel_torque")
    system = fresh_system(cfg)
    make_test_ground(system, cfg)
    rover = build_rover(system, spec, cfg, spawn_frame(0.0, 0.0))

    rover.set_command(10.0)  # τ_max 초과 명령 → 클램프
    assert rover.commanded_lr() == pytest.approx(
        (spec.max_wheel_torque_nm,) * 2)

    rover.set_command(0.5 * spec.max_wheel_torque_nm)
    while system.GetChTime() < 2.0:
        system.DoStepDynamics(cfg.timestep_s)
        rover.update(cfg.timestep_s)

    assert rover.forward_speed() > 0.1, "토크 모드에서 전진하지 않음"
    # 토크-속도 선형 맵: 무부하 속도(ω_max) 초과 시 back-EMF 제동 →
    # 정상상태 ω 는 ω_max 이하 (맵이 스텝당 1회 갱신되므로 0.5 % 이산화 여유)
    omega = sum(rover.wheel_omegas().values()) / 4
    assert omega <= spec.max_wheel_speed_radps * 1.005
    assert rover.energy_proxy_j() > 0


def test_spawn_settles_without_sinking_or_bouncing():
    """1.5 s 자유 안착: CG 높이 유지, 자세 수평 (sanity)."""
    spec, cfg = load_spec(SPEC_DIR / "scout_v01.yaml")
    system = fresh_system(cfg)
    make_test_ground(system, cfg)
    rover = build_rover(system, spec, cfg, spawn_frame(0.0, 0.0))

    while system.GetChTime() < 1.5:
        system.DoStepDynamics(cfg.timestep_s)
        rover.update(cfg.timestep_s)

    z = rover.chassis.GetPos().z
    assert abs(z - spec.cg_xyz_m[2]) < 0.005, f"z={z:.4f}, cg_z={spec.cg_xyz_m[2]}"
    roll, pitch, _ = rover.rpy_rad()
    assert abs(roll) < 0.01 and abs(pitch) < 0.01
