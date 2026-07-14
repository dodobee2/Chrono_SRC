"""로버 3D 시각화 뷰어 (VSG 우선, Irrlicht 폴백).

검증은 headless(run_rover_check.py)가 기본이고, 이 스크립트는 눈으로
로버 거동을 확인하고 싶을 때 쓰는 보조 도구다.

실행:
    conda activate chrono
    python scripts/view_rover.py                              # scout, 평지
    python scripts/view_rover.py --spec specs/rovers/main_v01.yaml
    python scripts/view_rover.py --slope 10                   # 10° 등판
    python scripts/view_rover.py --backend irrlicht           # 백엔드 강제

조작: 창을 닫으면 종료. 시작 2 s 안착 후 자동으로 정속 주행 명령이 들어간다.

macOS 주의 (연습 레포 규칙):
    - VSG 권장. Irrlicht 는 Retina 에서 1/4 렌더링 제한이 있음
    - vsync 미지원 폴백 대비 ChRealtimeStepTimer 로 실시간 동기화
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from chrono_env import ensure_chrono

ensure_chrono()

import pychrono as chrono  # noqa: E402

from rover_builder import apply_collision_defaults, build_rover  # noqa: E402
from rover_schema import load_spec  # noqa: E402
from test_ground import make_test_ground, spawn_frame  # noqa: E402

SETTLE_T = 2.0
RAMP_T = 1.0


def create_vis(system, backend: str, title: str):
    """VSG 우선 생성, 실패 시 Irrlicht (연습 레포 검증 패턴)."""
    order = ["vsg", "irrlicht"] if backend == "auto" else [backend]
    for b in order:
        if b == "vsg":
            try:
                import pychrono.vsg3d as chronovsg
            except ImportError:
                continue
            vis = chronovsg.ChVisualSystemVSG()
            vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
            vis.AttachSystem(system)
            vis.SetWindowSize(1280, 720)
            vis.SetWindowTitle(title)
            vis.AddCamera(chrono.ChVector3d(-1.2, -1.0, 0.6),
                          chrono.ChVector3d(0.0, 0.0, 0.05))
            vis.Initialize()  # VSG: AddCamera → Initialize 순서
            return vis, "VSG"
        if b == "irrlicht":
            try:
                import pychrono.irrlicht as chronoirr
            except ImportError:
                continue
            vis = chronoirr.ChVisualSystemIrrlicht()
            vis.AttachSystem(system)
            vis.SetWindowSize(1280, 720)
            vis.SetWindowTitle(title)
            vis.Initialize()  # Irrlicht: Initialize → 카메라/조명 순서
            vis.AddSkyBox()
            vis.AddCamera(chrono.ChVector3d(-1.2, -1.0, 0.6),
                          chrono.ChVector3d(0.0, 0.0, 0.05))
            vis.AddTypicalLights()
            return vis, "Irrlicht"
    raise SystemExit("사용 가능한 시각화 백엔드가 없습니다 (vsg3d/irrlicht).")


def main() -> None:
    ap = argparse.ArgumentParser(description="로버 3D 뷰어")
    ap.add_argument("--spec", type=Path,
                    default=PROJECT_ROOT / "specs/rovers/scout_v01.yaml")
    ap.add_argument("--slope", type=float, default=0.0, help="경사 [deg]")
    ap.add_argument("--speed-frac", type=float, default=0.6,
                    help="명령 크기 (max 대비 비율, 기본 0.6)")
    ap.add_argument("--duration", type=float, default=30.0, help="최대 시간 [s]")
    ap.add_argument("--backend", choices=("auto", "vsg", "irrlicht"),
                    default="auto")
    args = ap.parse_args()

    spec, cfg = load_spec(args.spec)
    apply_collision_defaults(cfg)
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -cfg.gravity_mps2))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    solver = system.GetSolver()
    if solver and hasattr(solver, "AsIterative"):
        it = solver.AsIterative()
        if it:
            it.SetMaxIterations(cfg.solver_max_iterations)

    make_test_ground(system, cfg, slope_deg=args.slope)
    x_start = -1.5 if args.slope else -2.0
    rover = build_rover(system, spec, cfg, spawn_frame(x_start, args.slope))

    if spec.command_type == "wheel_speed":
        target = args.speed_frac * spec.max_wheel_speed_radps
        cmd_desc = f"{target:.1f} rad/s (≈{target * spec.wheel_radius_m:.2f} m/s)"
    else:
        target = args.speed_frac * spec.max_wheel_torque_nm
        cmd_desc = f"{target * 1000:.1f} mN·m"

    vis, backend = create_vis(
        system, args.backend,
        f"{spec.rover_id} - slope {args.slope:g} deg")
    print(f"backend: {backend} | rover: {spec.rover_id} "
          f"({spec.mass_kg} kg) | 명령: {cmd_desc} (t={SETTLE_T:g}s 부터)")

    realtime = chrono.ChRealtimeStepTimer()
    dt = cfg.timestep_s
    while vis.Run() and system.GetChTime() < args.duration:
        t = system.GetChTime()
        ramp = min(1.0, max(0.0, (t - SETTLE_T) / RAMP_T))
        rover.set_command(target * ramp)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        system.DoStepDynamics(dt)
        rover.update(dt)

        # 추적 카메라 (지원되는 백엔드에서만)
        pos = rover.chassis.GetPos()
        if hasattr(vis, "SetCameraPosition"):
            vis.SetCameraPosition(chrono.ChVector3d(pos.x - 1.0, pos.y - 0.9, pos.z + 0.5))
        if hasattr(vis, "SetCameraTarget"):
            vis.SetCameraTarget(chrono.ChVector3d(pos.x + 0.2, pos.y, pos.z))

        realtime.Spin(dt)  # macOS: vsync 미지원 대비 실시간 동기화

    print(f"종료: t = {system.GetChTime():.2f} s, "
          f"x = {rover.chassis.GetPos().x:.3f} m, "
          f"v = {rover.forward_speed():.3f} m/s")


if __name__ == "__main__":
    main()
