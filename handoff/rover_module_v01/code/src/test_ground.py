"""내부 테스트용 평평한 rigid 바닥 (기울기 옵션 포함).

주의: 이것은 **로버 자체 검증용 테스트 픽스처**다. 실제 지형 생성은
지형 담당 팀원 모듈 소관이며, 팀 인터페이스와 무관하다.

기울기 옵션: slope_deg > 0 이면 바닥 전체를 Y축 기준 회전시켜
+X 방향이 오르막이 되게 한다. 바닥 윗면의 중심이 월드 원점을 지나도록
배치하므로, spawn_frame() 으로 얻은 프레임에 로버를 스폰하면
경사면 위에 정확히 안착한다.
"""

from __future__ import annotations

import math

import pychrono as chrono

from rover_schema import SimConfig


def make_test_ground(
    system: "chrono.ChSystemNSC",
    cfg: SimConfig | None = None,
    length: float = 6.0,
    width: float = 3.0,
    thickness: float = 0.1,
    slope_deg: float = 0.0,
) -> chrono.ChBody:
    """system 에 고정 바닥 박스를 추가하고 그 바디를 반환한다."""
    cfg = cfg or SimConfig()

    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(cfg.ground_friction)
    mat.SetRestitution(cfg.restitution)

    ground = chrono.ChBodyEasyBox(length, width, thickness, 1000.0, True, True, mat)
    ground.SetName(f"test_ground_slope{slope_deg:g}")
    ground.SetFixed(True)

    rot = chrono.QuatFromAngleY(math.radians(-slope_deg))  # +X 오르막
    ground.SetRot(rot)
    # 윗면 중심이 월드 원점에 오도록: center = R·(0, 0, −t/2)
    center = rot.Rotate(chrono.ChVector3d(0.0, 0.0, -thickness / 2.0))
    ground.SetPos(center)

    shape = ground.GetVisualShape(0)
    if shape:
        shape.SetColor(chrono.ChColor(0.45, 0.50, 0.45))
    system.AddBody(ground)
    return ground


def spawn_frame(x_local: float = 0.0, slope_deg: float = 0.0) -> chrono.ChFramed:
    """경사면 로컬 x 위치에 로버를 스폰하기 위한 프레임.

    반환 프레임의 XY 평면이 바닥 윗면과 일치한다.
    """
    rot = chrono.QuatFromAngleY(math.radians(-slope_deg))
    pos = rot.Rotate(chrono.ChVector3d(x_local, 0.0, 0.0))
    return chrono.ChFramed(pos, rot)
