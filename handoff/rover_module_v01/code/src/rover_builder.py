"""RoverSpec → PyChrono 로버 모델 빌더.

설계 원칙:
    - 빌더는 **외부에서 만든 chrono system 에 로버를 추가**한다 (시스템 소유 안 함).
      지형 담당 팀원의 모듈과 합칠 때 같은 system 을 공유하기 위함.
    - rigid 접촉 기반 MVP: 박스 섀시 + 실린더 휠 4개, 서스펜션 없음.
      SCM/DEM 확장 시 접촉 재질/휠 생성만 교체하면 되도록 분리해 둔다.
    - 관성은 치수 기반 근사 (박스/솔리드 실린더 공식).
    - 모델 총질량 == spec.mass_kg 를 build 시점에 assert.

좌표계: rover_schema.py 의 규약 참조 (Z-up, X 전진, Y 좌측,
원점 = 4휠 접지점 직사각형 중심의 접지면 투영점).

명령 처리 (command_type):
    - "wheel_speed":  ChLinkMotorRotationSpeed. 명령 ω 는 ±max_wheel_speed 로
      클램프. 요구 토크는 로그로 노출 (이상적인 속도 모터라 토크 한계는
      물리적으로 강제되지 않음 → 검증 단계에서 초과 여부를 감시).
    - "wheel_torque": ChLinkMotorRotationTorque. 인가 토크는 DC 모터
      토크-속도 선형 맵  τ_avail(ω) = τ_max · (1 − |ω|/ω_max)  로 클램프.

휠 회전 부호 규약: 모터 프레임을 X축 −90° 회전으로 두어 (연습 코드 검증 패턴)
양(+)의 모터 속도/토크 = 로버 +X 전진 방향.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import pychrono as chrono

from rover_schema import RoverSpec, SimConfig, validate_spec

WHEEL_NAMES = ("FL", "FR", "RL", "RR")  # Front/Rear × Left/Right


def _make_const(value: float) -> chrono.ChFunctionConst:
    return chrono.ChFunctionConst(value)


def _wheel_local_positions(spec: RoverSpec) -> dict[str, chrono.ChVector3d]:
    """로버 로컬 좌표계에서 휠 중심 위치."""
    hx = spec.wheelbase_m / 2.0
    hy = spec.track_width_m / 2.0
    z = spec.wheel_radius_m
    return {
        "FL": chrono.ChVector3d(+hx, +hy, z),
        "FR": chrono.ChVector3d(+hx, -hy, z),
        "RL": chrono.ChVector3d(-hx, +hy, z),
        "RR": chrono.ChVector3d(-hx, -hy, z),
    }


def _driven_wheel_names(spec: RoverSpec) -> tuple[str, ...]:
    if spec.driven_wheel_count == 4:
        return WHEEL_NAMES
    # ASSUMPTION: 2WD 인 경우 뒷바퀴 구동으로 가정 — 팀장 확인 필요
    return ("RL", "RR")


def chassis_dimensions(spec: RoverSpec, cfg: SimConfig) -> tuple[float, float, float]:
    """섀시 박스 치수. SimConfig 에 명시가 없으면 spec 에서 유도한다.

    높이는 박스 중심이 CG 높이에 오면서 바닥면이 지상고에 닿도록
    height = 2*(cg_z − ground_clearance) 로 잡는다 (CG 일치 목적의 근사).
    """
    if cfg.chassis_size_m is not None:
        return cfg.chassis_size_m
    length = cfg.chassis_length_ratio * spec.wheelbase_m
    width = spec.track_width_m - spec.wheel_width_m - 2.0 * cfg.wheel_gap_m
    height = 2.0 * (spec.cg_xyz_m[2] - spec.ground_clearance_m)
    if width <= 0 or height <= 0:
        raise ValueError(
            f"유도된 섀시 치수가 비물리적입니다 (width={width:.4f}, height={height:.4f}). "
            "SimConfig.chassis_size_m 로 직접 지정하세요."
        )
    return (length, width, height)


def apply_collision_defaults(cfg: SimConfig) -> None:
    """전역 충돌 여유 설정. **바디 생성 전에** 한 번 호출해야 한다."""
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(cfg.collision_envelope_m)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(cfg.collision_margin_m)


@dataclass
class RoverInstance:
    """build_rover 결과물: 바디/모터 핸들과 명령/상태 API."""

    spec: RoverSpec
    config: SimConfig
    chassis: chrono.ChBody
    wheels: dict[str, chrono.ChBody]
    motors: dict[str, "chrono.ChLinkMotorRotation"]      # 구동 휠만
    free_joints: dict[str, chrono.ChLinkLockRevolute]    # 비구동 휠
    _cmd: dict[str, float] = field(default_factory=dict)  # 휠별 명령값
    _energy_j: float = field(default=0.0)                # ∫ Σ|τω| dt (에너지 proxy)

    # ── 명령 ────────────────────────────────────────────────────────────
    def _clamp_command(self, value: float) -> float:
        lim = (self.spec.max_wheel_speed_radps
               if self.spec.command_type == "wheel_speed"
               else self.spec.max_wheel_torque_nm)
        return max(-lim, min(lim, value))

    def _apply_wheel_command(self, name: str, value: float) -> None:
        self._cmd[name] = value
        if self.spec.command_type == "wheel_speed":
            self.motors[name].SetSpeedFunction(_make_const(value))
        # wheel_torque: 실제 인가 토크는 update() 에서 토크-속도 맵으로 결정

    def set_command(self, value: float) -> None:
        """전 구동휠 동일 명령 (직진).

        wheel_speed  → value = 목표 휠 각속도 [rad/s] (±max 클램프)
        wheel_torque → value = 목표 휠 토크 [N·m] (선형 맵 클램프는 update() 에서)
        """
        v = self._clamp_command(value)
        for name in self.motors:
            self._apply_wheel_command(name, v)

    def set_command_lr(self, left: float, right: float) -> None:
        """좌/우 독립 명령 (skid steer 선회).

        좌우 바퀴 속도 차로 방향을 바꾼다. left < right 면 좌회전(yaw +),
        left = -right 면 제자리 선회. 휠 이름 규약: ?L = 좌측(+Y), ?R = 우측.
        """
        lv, rv = self._clamp_command(left), self._clamp_command(right)
        for name in self.motors:
            self._apply_wheel_command(name, lv if name.endswith("L") else rv)

    def commanded_lr(self) -> tuple[float, float]:
        """현재 (좌, 우) 명령값 (로깅용). 명령 전이면 (0, 0)."""
        left = [v for n, v in self._cmd.items() if n.endswith("L")]
        right = [v for n, v in self._cmd.items() if n.endswith("R")]
        return (left[0] if left else 0.0, right[0] if right else 0.0)

    def _dc_motor_torque(self, cmd: float, omega: float) -> float:
        """DC 모터 토크-속도 선형 맵.

        곡선 한계 τ_curve(ω) = ±τ_max·(1 ∓ ω/ω_max) — 무부하 속도(ω_max)를
        넘어서면 음수가 되어 back-EMF 제동을 모사한다. 인가 토크는
        min(명령, 곡선 한계) 후 ±τ_max 로 최종 클램프.
        """
        tmax = self.spec.max_wheel_torque_nm
        wmax = self.spec.max_wheel_speed_radps
        if cmd >= 0.0:
            tau = min(cmd, tmax * (1.0 - omega / wmax))
        else:
            tau = max(cmd, -tmax * (1.0 + omega / wmax))
        return max(-tmax, min(tmax, tau))

    def update(self, dt: float) -> None:
        """매 스텝 호출: 토크-속도 맵 적용(torque 모드) + 에너지 적분."""
        if self.spec.command_type == "wheel_torque":
            for name, m in self.motors.items():
                omega = self.wheel_speed_of(m)
                tau = self._dc_motor_torque(self._cmd.get(name, 0.0), omega)
                m.SetTorqueFunction(_make_const(tau))
        self._energy_j += self.total_power_w() * dt

    # ── 상태 조회 ────────────────────────────────────────────────────────
    @staticmethod
    def wheel_speed_of(motor) -> float:
        """모터 상대 각속도 [rad/s] (버전별 API 명 차이 흡수)."""
        for name in ("GetMotorAngleDt", "GetMotorRot_dt", "GetMotorRotDt"):
            if hasattr(motor, name):
                return float(getattr(motor, name)())
        raise AttributeError("모터 각속도 API 를 찾지 못함")

    def wheel_omegas(self) -> dict[str, float]:
        out = {}
        for name in WHEEL_NAMES:
            if name in self.motors:
                out[name] = self.wheel_speed_of(self.motors[name])
            else:
                # 비구동 휠: 섀시 대비 상대 각속도 (Y축 성분)
                rel = self.wheels[name].GetAngVelLocal()
                out[name] = float(rel.y)
        return out

    def wheel_torques(self) -> dict[str, float]:
        return {n: float(m.GetMotorTorque()) for n, m in self.motors.items()}

    def forward_speed(self) -> float:
        """섀시 전진(+X body) 속도 [m/s]."""
        v_world = self.chassis.GetPosDt()
        x_axis = self.chassis.GetRot().Rotate(chrono.ChVector3d(1, 0, 0))
        return float(v_world ^ x_axis)  # dot product

    def slip_ratios(self) -> dict[str, float]:
        """구동 슬립률: (rω − v)/max(|rω|, ε). 정지 근처에서는 0 처리."""
        r = self.spec.wheel_radius_m
        v = self.forward_speed()
        eps = 1e-3
        out = {}
        for name, omega in self.wheel_omegas().items():
            denom = max(abs(r * omega), eps)
            slip = (r * omega - v) / denom
            if abs(r * omega) < eps and abs(v) < eps:
                slip = 0.0
            out[name] = slip
        return out

    def total_power_w(self) -> float:
        """Σ|τ_w · ω_w| — 기계적 출력 크기 합 (에너지 proxy 용)."""
        omegas = self.wheel_omegas()
        return sum(
            abs(tau * omegas[name]) for name, tau in self.wheel_torques().items()
        )

    def energy_proxy_j(self) -> float:
        return self._energy_j

    def rpy_rad(self) -> tuple[float, float, float]:
        """(roll, pitch, yaw) [rad] — Cardan XYZ."""
        e = self.chassis.GetRot().GetCardanAnglesXYZ()
        return float(e.x), float(e.y), float(e.z)

    def total_mass(self) -> float:
        return float(
            self.chassis.GetMass() + sum(w.GetMass() for w in self.wheels.values())
        )


def overall_dimensions(spec: RoverSpec, cfg: SimConfig | None = None
                       ) -> tuple[float, float, float]:
    """주행 자세 기준 전장 × 전폭 × 전고 [m] (마스트 제외 — 수납 시 접이 가정).

    공고문 수납부피(300×300×200 mm) 대비 여유 확인용.
    """
    cfg = cfg or SimConfig()
    L, W, H = chassis_dimensions(spec, cfg)
    length = max(L, spec.wheelbase_m + 2.0 * spec.wheel_radius_m)
    width = max(W, spec.track_width_m + spec.wheel_width_m)
    height = max(2.0 * spec.wheel_radius_m, spec.ground_clearance_m + H)
    return (length, width, height)


def _add_chassis_visuals(chassis: chrono.ChBody, spec: RoverSpec,
                         cfg: SimConfig, dims: tuple[float, float, float]
                         ) -> None:
    """대회 임무 장비를 나타내는 시각 요소 (물리 접촉·질량 없음).

    카메라/안테나 마스트(촬영·통신), 전방 depth 센서(지형 스캔),
    상판 데크(전장부 커버)를 표현한다. 질량은 섀시 박스가 대표하므로
    CG·총질량은 spec 값 그대로 유지된다.
    """
    L, W, H = dims
    top = H / 2.0

    deck = chrono.ChVisualShapeBox(0.85 * L, 0.85 * W, 0.006)
    deck.SetColor(chrono.ChColor(0.78, 0.76, 0.68))
    chassis.AddVisualShape(deck, chrono.ChFramed(chrono.ChVector3d(0, 0, top + 0.003)))

    # 카메라/안테나 마스트 (기본 축 Z = 수직, 회전 불필요)
    mast_h = cfg.mast_height_m
    mast = chrono.ChVisualShapeCylinder(0.006, mast_h)
    mast.SetColor(chrono.ChColor(0.72, 0.74, 0.78))
    mast_x = -0.30 * L                                 # 후방 쪽에 세움
    chassis.AddVisualShape(
        mast, chrono.ChFramed(chrono.ChVector3d(mast_x, 0, top + mast_h / 2.0)))

    head = chrono.ChVisualShapeBox(0.045, 0.06, 0.03)  # 카메라 헤드
    head.SetColor(chrono.ChColor(0.92, 0.66, 0.16))
    chassis.AddVisualShape(
        head, chrono.ChFramed(chrono.ChVector3d(mast_x + 0.01, 0, top + mast_h + 0.015)))

    sensor = chrono.ChVisualShapeBox(0.015, 0.5 * W, 0.02)  # 전방 depth 센서
    sensor.SetColor(chrono.ChColor(0.15, 0.15, 0.18))
    chassis.AddVisualShape(
        sensor, chrono.ChFramed(chrono.ChVector3d(L / 2.0 + 0.0075, 0, top - 0.02)))


def _add_wheel_visuals(wheel: chrono.ChBody, spec: RoverSpec,
                       cfg: SimConfig) -> None:
    """휠 허브 + 그루저 시각 요소 (접촉은 원통 휠 바디가 담당)."""
    r, w = spec.wheel_radius_m, spec.wheel_width_m
    axis_rot = chrono.QuatFromAngleX(math.pi / 2.0)    # 실린더 Z축 → Y축(차축)

    hub = chrono.ChVisualShapeCylinder(0.35 * r, 1.08 * w)
    hub.SetColor(chrono.ChColor(0.75, 0.75, 0.78))
    wheel.AddVisualShape(hub, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), axis_rot))

    for i in range(cfg.grouser_count):
        ang = 2.0 * math.pi * i / cfg.grouser_count
        rg = 0.98 * r
        grouser = chrono.ChVisualShapeBox(0.010, 1.04 * w, 0.006)
        grouser.SetColor(chrono.ChColor(0.30, 0.30, 0.32))
        pos = chrono.ChVector3d(rg * math.cos(ang), 0.0, rg * math.sin(ang))
        wheel.AddVisualShape(
            grouser, chrono.ChFramed(pos, chrono.QuatFromAngleY(-ang)))


def build_rover(
    system: "chrono.ChSystemNSC",
    spec: RoverSpec,
    cfg: SimConfig | None = None,
    spawn_frame: "chrono.ChFramed | None" = None,
    color: tuple[float, float, float] = (0.12, 0.38, 0.82),
) -> RoverInstance:
    """외부 system 에 RoverSpec 대로 로버를 추가한다.

    Parameters
    ----------
    system : 외부에서 생성/소유하는 Chrono 시스템 (빌더는 소유하지 않음)
    spawn_frame : 로버 로컬 원점(접지면 투영 중심)을 월드에 배치할 프레임.
        None 이면 원점. 경사면 스폰 시 test_ground.spawn_frame() 사용.
    """
    validate_spec(spec)
    cfg = cfg or SimConfig()
    if spawn_frame is None:
        spawn_frame = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)

    # 스폰 낙하 여유: 로컬 +Z 로 살짝 띄운다
    drop = chrono.ChVector3d(0, 0, cfg.spawn_drop_m)

    def to_world_pos(local: chrono.ChVector3d) -> chrono.ChVector3d:
        return spawn_frame.TransformPointLocalToParent(local + drop)

    world_rot = spawn_frame.GetRot()

    wheel_mat = chrono.ChContactMaterialNSC()
    wheel_mat.SetFriction(cfg.wheel_friction)
    wheel_mat.SetRestitution(cfg.restitution)

    # ── 질량 분배 ────────────────────────────────────────────────────────
    wheel_mass = (
        cfg.wheel_mass_kg
        if cfg.wheel_mass_kg is not None
        else cfg.wheel_mass_fraction * spec.mass_kg
    )
    chassis_mass = spec.mass_kg - spec.wheel_count * wheel_mass
    if chassis_mass <= 0:
        raise ValueError(
            f"휠 질량 합({spec.wheel_count * wheel_mass:.3f} kg)이 "
            f"총질량({spec.mass_kg} kg)을 초과합니다. SimConfig 확인."
        )

    # ── 섀시: 박스, 중심 = CG (모델 CG 를 spec 과 일치시키는 근사) ─────────
    L, W, H = chassis_dimensions(spec, cfg)
    cg = chrono.ChVector3d(*spec.cg_xyz_m)

    chassis = chrono.ChBodyEasyBox(L, W, H, 1000.0, True, True, wheel_mat)
    chassis.SetName(f"{spec.rover_id}_chassis")
    chassis.SetMass(chassis_mass)
    # 박스 관성 근사: I = m/12 · (a²+b²)
    chassis.SetInertiaXX(chrono.ChVector3d(
        chassis_mass / 12.0 * (W * W + H * H),
        chassis_mass / 12.0 * (L * L + H * H),
        chassis_mass / 12.0 * (L * L + W * W),
    ))
    chassis.SetPos(to_world_pos(cg))
    chassis.SetRot(world_rot)
    shape = chassis.GetVisualShape(0)
    if shape:
        shape.SetColor(chrono.ChColor(*color))
    if cfg.detail_visuals:
        _add_chassis_visuals(chassis, spec, cfg, (L, W, H))
    system.AddBody(chassis)

    # ── 휠: 실린더 (Y축 회전축), 솔리드 실린더 관성 근사 ─────────────────
    wheel_positions = _wheel_local_positions(spec)
    driven = _driven_wheel_names(spec)
    r, w = spec.wheel_radius_m, spec.wheel_width_m
    Iyy = 0.5 * wheel_mass * r * r                       # 회전축
    Ixx = wheel_mass / 12.0 * (3.0 * r * r + w * w)      # 지름축

    wheels: dict[str, chrono.ChBody] = {}
    motors: dict = {}
    free_joints: dict[str, chrono.ChLinkLockRevolute] = {}

    # 모터/조인트 프레임: Y축이 회전축이 되도록 X축 −90° 회전 (검증 패턴)
    axis_rot = chrono.QuatFromAngleX(-math.pi / 2.0)

    for name in WHEEL_NAMES:
        local = wheel_positions[name]
        wheel = chrono.ChBodyEasyCylinder(
            chrono.ChAxis_Y, r, w, 1000.0, True, True, wheel_mat
        )
        wheel.SetName(f"{spec.rover_id}_wheel_{name}")
        wheel.SetMass(wheel_mass)
        wheel.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Ixx))
        wheel.SetPos(to_world_pos(local))
        wheel.SetRot(world_rot)
        wshape = wheel.GetVisualShape(0)
        if wshape:
            wshape.SetColor(chrono.ChColor(0.10, 0.10, 0.10))
        if cfg.detail_visuals:
            _add_wheel_visuals(wheel, spec, cfg)
        system.AddBody(wheel)
        wheels[name] = wheel

        joint_frame = chrono.ChFramed(to_world_pos(local), world_rot * axis_rot)

        if name in driven:
            if spec.command_type == "wheel_speed":
                motor = chrono.ChLinkMotorRotationSpeed()
                motor.SetSpeedFunction(_make_const(0.0))
            else:
                motor = chrono.ChLinkMotorRotationTorque()
                motor.SetTorqueFunction(_make_const(0.0))
            motor.SetName(f"{spec.rover_id}_motor_{name}")
            motor.Initialize(wheel, chassis, joint_frame)
            system.Add(motor)
            motors[name] = motor
        else:
            rev = chrono.ChLinkLockRevolute()
            rev.SetName(f"{spec.rover_id}_free_{name}")
            rev.Initialize(wheel, chassis, joint_frame)
            system.Add(rev)
            free_joints[name] = rev

    rover = RoverInstance(
        spec=spec, config=cfg, chassis=chassis,
        wheels=wheels, motors=motors, free_joints=free_joints,
    )

    # 완료 기준: 빌더가 만든 모델 총질량 == spec.mass_kg
    assert abs(rover.total_mass() - spec.mass_kg) < 1e-6, (
        f"총질량 불일치: model={rover.total_mass():.6f} kg, "
        f"spec={spec.mass_kg} kg"
    )
    return rover
