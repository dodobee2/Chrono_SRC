"""RoverSpec 데이터 계약 + yaml 로더/검증, SimConfig.

단위: 전부 SI (m, kg, s, rad, N·m).

좌표계 규약 (ASSUMPTION — 팀장 확인 필요):
    - Z-up, X 전진, Y 좌측 (오른손 좌표계). Chrono Z-up 관례와 동일.
    - RoverSpec 의 원점 = 4개 휠 접지점이 이루는 직사각형의 중심을
      지면(접지 평면)에 투영한 점. 정지 상태 기준.
    - cg_xyz_m 은 이 원점에서 본 로버 전체 CG 위치.
      예: (0, 0, 0.05) = 휠 접지 중심 바로 위 5 cm.
    - wheelbase_m 은 앞/뒤 휠 축간 X 거리, track_width_m 은 좌/우 휠 중심 Y 거리.

RoverSpec 은 팀장이 정의한 계약이므로 필드를 수정하지 않는다.
Chrono 구현에 필요하지만 계약에 없는 값은 SimConfig 로 분리한다.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path

import yaml

VALID_COMMAND_TYPES = ("wheel_speed", "wheel_torque")


# ── 팀장 정의 데이터 계약 (수정 금지) ────────────────────────────────────────
@dataclass
class RoverSpec:
    rover_id: str
    mass_kg: float
    cg_xyz_m: tuple[float, float, float]
    wheel_count: int
    driven_wheel_count: int
    wheel_radius_m: float
    wheel_width_m: float
    wheelbase_m: float
    track_width_m: float
    ground_clearance_m: float
    command_type: str            # "wheel_speed" | "wheel_torque"
    max_wheel_torque_nm: float   # 휠당
    max_wheel_speed_radps: float


# ── Chrono 구현용 부가 설정 (계약 외 값, 기본값 + yaml override) ─────────────
@dataclass
class SimConfig:
    """빌더/시뮬레이션 내부 기본값. specs yaml 의 `sim:` 블록으로 override 가능."""

    # 질량 분배: 휠 하나의 질량. None 이면 총질량의 wheel_mass_fraction 로 계산.
    wheel_mass_kg: float | None = None
    wheel_mass_fraction: float = 0.05      # 휠당 (4륜 기준 총 20 %)

    # 섀시 박스 치수 [m]. None 이면 spec 에서 유도:
    #   length = chassis_length_ratio * wheelbase
    #   width  = track_width - wheel_width - 2*wheel_gap
    #   height = 2 * (cg_z - ground_clearance)  (박스 중심 = CG 가 되도록)
    chassis_size_m: tuple[float, float, float] | None = None
    chassis_length_ratio: float = 0.9
    wheel_gap_m: float = 0.005             # 섀시-휠 안쪽면 간격

    # 접촉 재질 (NSC)
    wheel_friction: float = 0.8
    ground_friction: float = 0.8
    restitution: float = 0.05

    # 충돌 여유 (소형 휠 대비 기본값이 크므로 축소; 연습 코드 검증값)
    collision_envelope_m: float = 0.0025
    collision_margin_m: float = 0.0025

    # 시뮬레이션
    timestep_s: float = 2.0e-3
    solver_max_iterations: int = 150
    gravity_mps2: float = 9.81             # 지구 기준. 달 1.62 / 화성 3.71

    # 스폰 시 지면 위 낙하 여유 [m]
    spawn_drop_m: float = 0.002

    # 외형 시각 요소 (물리 접촉 없음 — 카메라 마스트, 센서 헤드, 그루저 등)
    # 질량은 전부 섀시 박스에 포함되므로 CG/총질량 계산에 영향 없음
    detail_visuals: bool = True
    mast_height_m: float = 0.12            # 카메라/안테나 마스트 (수납 시 접이 가정)
    grouser_count: int = 12                # 휠당 그루저 개수 (시각 표현)


class SpecValidationError(ValueError):
    """RoverSpec yaml 검증 실패."""


def _check(cond: bool, msg: str, errors: list[str]) -> None:
    if not cond:
        errors.append(msg)


def validate_spec(spec: RoverSpec) -> None:
    """물리적/논리적 정합성 검증. 실패 시 SpecValidationError."""
    e: list[str] = []
    _check(bool(spec.rover_id), "rover_id 가 비어 있음", e)
    _check(spec.mass_kg > 0, f"mass_kg > 0 이어야 함 (got {spec.mass_kg})", e)
    _check(
        isinstance(spec.cg_xyz_m, tuple) and len(spec.cg_xyz_m) == 3,
        f"cg_xyz_m 은 길이 3 tuple 이어야 함 (got {spec.cg_xyz_m!r})", e,
    )
    _check(spec.wheel_count == 4,
           f"현재 빌더는 4륜만 지원 (got wheel_count={spec.wheel_count})", e)
    _check(0 < spec.driven_wheel_count <= spec.wheel_count,
           f"driven_wheel_count 는 1..wheel_count (got {spec.driven_wheel_count})", e)
    _check(spec.driven_wheel_count in (2, 4),
           f"driven_wheel_count 는 2 또는 4 지원 (got {spec.driven_wheel_count})", e)
    _check(spec.wheel_radius_m > 0, "wheel_radius_m > 0", e)
    _check(spec.wheel_width_m > 0, "wheel_width_m > 0", e)
    _check(spec.wheelbase_m > 0, "wheelbase_m > 0", e)
    _check(spec.track_width_m > 0, "track_width_m > 0", e)
    _check(spec.ground_clearance_m > 0, "ground_clearance_m > 0", e)
    _check(spec.ground_clearance_m < spec.wheel_radius_m * 2,
           "ground_clearance 가 휠 직경보다 큼 — 값 확인 필요", e)
    _check(spec.command_type in VALID_COMMAND_TYPES,
           f"command_type 은 {VALID_COMMAND_TYPES} 중 하나 (got {spec.command_type!r})", e)
    _check(spec.max_wheel_torque_nm > 0, "max_wheel_torque_nm > 0", e)
    _check(spec.max_wheel_speed_radps > 0, "max_wheel_speed_radps > 0", e)

    if len(spec.cg_xyz_m) == 3:
        cg_z = spec.cg_xyz_m[2]
        _check(cg_z > spec.ground_clearance_m,
               f"CG 높이({cg_z}) 는 지상고({spec.ground_clearance_m})보다 높아야 함", e)
        _check(spec.track_width_m > spec.wheel_width_m,
               "track_width 는 wheel_width 보다 커야 함", e)

    if e:
        raise SpecValidationError(
            f"RoverSpec '{spec.rover_id}' 검증 실패:\n  - " + "\n  - ".join(e)
        )


def _spec_from_dict(d: dict) -> RoverSpec:
    spec_fields = {f.name for f in fields(RoverSpec)}
    unknown = set(d) - spec_fields
    if unknown:
        raise SpecValidationError(f"알 수 없는 필드: {sorted(unknown)}")
    missing = spec_fields - set(d)
    if missing:
        raise SpecValidationError(f"누락된 필드: {sorted(missing)}")

    d = dict(d)
    cg = d["cg_xyz_m"]
    if not isinstance(cg, (list, tuple)) or len(cg) != 3:
        raise SpecValidationError(f"cg_xyz_m 은 [x, y, z] 형식 (got {cg!r})")
    d["cg_xyz_m"] = tuple(float(v) for v in cg)

    for f in fields(RoverSpec):
        if f.name in ("rover_id", "command_type", "cg_xyz_m"):
            continue
        try:
            d[f.name] = (int(d[f.name]) if f.type == "int" else float(d[f.name]))
        except (TypeError, ValueError):
            raise SpecValidationError(
                f"{f.name} 숫자 변환 실패 (got {d[f.name]!r})"
            ) from None
    return RoverSpec(**d)


def _sim_config_from_dict(d: dict) -> SimConfig:
    cfg_fields = {f.name for f in fields(SimConfig)}
    unknown = set(d) - cfg_fields
    if unknown:
        raise SpecValidationError(f"sim 블록에 알 수 없는 필드: {sorted(unknown)}")
    if "chassis_size_m" in d and d["chassis_size_m"] is not None:
        d = dict(d)
        d["chassis_size_m"] = tuple(float(v) for v in d["chassis_size_m"])
    return SimConfig(**d)


def load_spec(path: str | Path) -> tuple[RoverSpec, SimConfig]:
    """yaml 파일에서 (RoverSpec, SimConfig) 를 로드·검증한다.

    yaml 구조:
        rover:   # RoverSpec 필드 (필수)
          rover_id: ...
        sim:     # SimConfig override (선택)
          timestep_s: ...
    """
    path = Path(path)
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict) or "rover" not in data:
        raise SpecValidationError(f"{path}: 최상위에 'rover' 블록이 필요함")

    spec = _spec_from_dict(data["rover"])
    validate_spec(spec)
    cfg = _sim_config_from_dict(data.get("sim") or {})
    return spec, cfg
