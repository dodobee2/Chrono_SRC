"""로버 렌더 이미지 캡처 (팀 공유/제안서용 PNG).

씬을 만들고 잠시 안착시킨 뒤 지정 각도에서 스크린샷을 저장한다.
실행하면 창이 잠깐 떴다 닫힌다.

    conda activate chrono
    python scripts/render_rover.py                 # scout, main, 나란히 3장
    python scripts/render_rover.py --out-dir outputs/renders
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
from rover_schema import SimConfig, load_spec  # noqa: E402
from test_ground import make_test_ground  # noqa: E402

SPEC_DIR = PROJECT_ROOT / "specs" / "rovers"

SCOUT_COLOR = (0.13, 0.42, 0.80)   # 파랑
MAIN_COLOR = (0.80, 0.25, 0.18)    # 빨강


def make_system(cfg: SimConfig) -> chrono.ChSystemNSC:
    apply_collision_defaults(cfg)
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -cfg.gravity_mps2))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    return system


def offset_frame(y: float) -> chrono.ChFramed:
    return chrono.ChFramed(chrono.ChVector3d(0.0, y, 0.0), chrono.QUNIT)


def render_scene(builders, cam_pos, cam_target, out_png: Path,
                 settle_s: float = 1.0) -> None:
    """builders: [(spec, cfg, spawn_y, color), ...] 를 한 씬에 렌더."""
    cfg0 = builders[0][1]
    system = make_system(cfg0)
    make_test_ground(system, cfg0, length=4.0, width=2.5)
    rovers = [
        build_rover(system, spec, cfg, offset_frame(y), color=color)
        for spec, cfg, y, color in builders
    ]

    import pychrono.vsg3d as chronovsg
    vis = chronovsg.ChVisualSystemVSG()
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.AttachSystem(system)
    vis.SetWindowSize(1600, 1000)
    vis.SetWindowTitle("render")
    vis.AddCamera(chrono.ChVector3d(*cam_pos), chrono.ChVector3d(*cam_target))
    vis.SetBaseGuiVisibility(False)   # 통계 패널 숨김 (보고용 캡처)
    vis.Initialize()

    # 안착 후 몇 프레임 그리고 나서 캡처 (첫 프레임은 초기화 직후라 피함)
    dt = cfg0.timestep_s
    while system.GetChTime() < settle_s and vis.Run():
        system.DoStepDynamics(dt)
        for r in rovers:
            r.update(dt)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    out_png.parent.mkdir(parents=True, exist_ok=True)
    vis.WriteImageToFile(str(out_png))
    # 일부 백엔드는 다음 렌더 시점에 파일을 쓴다
    for _ in range(3):
        if not vis.Run():
            break
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    print(f"render: {out_png}")


def run_scene(scene: str, out_dir: Path) -> None:
    scout, scout_cfg = load_spec(SPEC_DIR / "scout_v01.yaml")
    main_, main_cfg = load_spec(SPEC_DIR / "main_v01.yaml")

    if scene == "scout":  # 단독 샷: 낮은 3/4 앵글
        render_scene(
            [(scout, scout_cfg, 0.0, SCOUT_COLOR)],
            cam_pos=(-0.38, -0.33, 0.18), cam_target=(0.0, 0.0, 0.045),
            out_png=out_dir / "scout_v01.png",
        )
    elif scene == "main":
        render_scene(
            [(main_, main_cfg, 0.0, MAIN_COLOR)],
            cam_pos=(-0.50, -0.45, 0.25), cam_target=(0.0, 0.0, 0.06),
            out_png=out_dir / "main_v01.png",
        )
    elif scene == "both":  # 나란히: 크기 비교
        render_scene(
            [(scout, scout_cfg, -0.18, SCOUT_COLOR),
             (main_, main_cfg, +0.18, MAIN_COLOR)],
            cam_pos=(-0.65, -0.55, 0.35), cam_target=(0.0, 0.0, 0.05),
            out_png=out_dir / "scout_vs_main.png",
        )


def main() -> None:
    ap = argparse.ArgumentParser(description="로버 렌더 캡처")
    ap.add_argument("--out-dir", type=Path,
                    default=PROJECT_ROOT / "outputs" / "renders")
    ap.add_argument("--scene", choices=("scout", "main", "both"),
                    help="지정 시 해당 씬만 렌더 (내부용)")
    args = ap.parse_args()

    if args.scene:
        run_scene(args.scene, args.out_dir)
        return

    # VSG 는 한 프로세스에서 창을 두 번 만들면 크래시하므로 씬마다 서브프로세스
    import subprocess
    for scene in ("scout", "main", "both"):
        r = subprocess.run(
            [sys.executable, __file__, "--scene", scene,
             "--out-dir", str(args.out_dir)],
        )
        if r.returncode != 0:
            print(f"경고: scene '{scene}' 렌더 실패 (exit {r.returncode})")


if __name__ == "__main__":
    main()
