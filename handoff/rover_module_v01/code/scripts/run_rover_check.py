"""로버 자체 검증 스크립트 (headless).

검증 항목 (goal.md 완료 기준):
  A. 스폰: 바닥 안착 후 정지 유지 (튀거나 가라앉지 않음)
  B. 평지 정속 주행: 직진 유지, 슬립률 ≈ 0, 상용 속도 도달
  C. 경사 등판: 토크 요구 증가 (기대치 ≈ m·g·sinθ·r/4 와 비교)
  D. scout vs main: 질량·CG·휠 차이가 토크/에너지에 반영되는지

실행:
    conda activate chrono
    python scripts/run_rover_check.py                 # 두 스펙 전체 검증
    python scripts/run_rover_check.py --spec specs/rovers/scout_v01.yaml
    python scripts/run_rover_check.py --slope 15 --no-plot

결과: outputs/rover_check/<rover_id>/*.csv + *.png, summary.json
종료 코드: 모든 체크 통과 0, 실패 있으면 1
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from chrono_env import ensure_chrono

ensure_chrono()

import pychrono as chrono  # noqa: E402

from logger import RoverLogger, read_csv  # noqa: E402
from rover_builder import (  # noqa: E402
    apply_collision_defaults,
    build_rover,
    overall_dimensions,
)
from rover_schema import RoverSpec, SimConfig, load_spec  # noqa: E402
from test_ground import make_test_ground, spawn_frame  # noqa: E402

DEFAULT_SPECS = [
    PROJECT_ROOT / "specs/rovers/scout_v01.yaml",
    PROJECT_ROOT / "specs/rovers/main_v01.yaml",
]
OUT_ROOT = PROJECT_ROOT / "outputs" / "rover_check"

SETTLE_T = 2.0        # [s] 스폰 안착 관찰 시간
CRUISE_T = 8.0        # [s] 정속 주행 시간 (ramp 포함)
RAMP_T = 1.0          # [s] 명령 램프 시간
LOG_DT = 0.02         # [s] CSV 기록 간격


def make_system(cfg: SimConfig) -> chrono.ChSystemNSC:
    """검증용 시스템. (실전에서는 지형 담당 모듈이 system 을 소유한다)"""
    apply_collision_defaults(cfg)  # 바디 생성 전에 호출
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(
        chrono.ChVector3d(0, 0, -cfg.gravity_mps2))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    solver = system.GetSolver()
    if solver and hasattr(solver, "AsIterative"):
        it = solver.AsIterative()
        if it:
            it.SetMaxIterations(cfg.solver_max_iterations)
    return system


def cruise_command(spec: RoverSpec) -> float:
    """상용 명령값: wheel_speed 는 0.6·ω_max, wheel_torque 는 0.5·τ_max."""
    if spec.command_type == "wheel_speed":
        return 0.6 * spec.max_wheel_speed_radps
    return 0.5 * spec.max_wheel_torque_nm


def simulate(spec, cfg, slope_deg: float, mode: str, duration: float,
             csv_path: Path, x_start: float = 0.0) -> dict[str, list[float]]:
    """한 시나리오를 실행하고 CSV 저장 + 데이터 반환.

    mode: "settle"(무명령) | "cruise"(직진) | "pivot"(제자리 선회)
    """
    system = make_system(cfg)
    make_test_ground(system, cfg, slope_deg=slope_deg)
    rover = build_rover(system, spec,
                        cfg, spawn_frame(x_start, slope_deg))

    target = cruise_command(spec) if mode != "settle" else 0.0
    dt = cfg.timestep_s
    next_log = 0.0
    settle = SETTLE_T if mode != "settle" else 0.0

    with RoverLogger(csv_path, rover) as log:
        t = 0.0
        while t < duration + settle:
            if mode != "settle":
                ramp = min(1.0, max(0.0, (t - settle) / RAMP_T))
                if mode == "pivot":
                    # 좌우 역회전 → 제자리 좌선회 (yaw +)
                    v = 0.5 * target * ramp
                    rover.set_command_lr(-v, v)
                else:
                    rover.set_command(target * ramp)
            system.DoStepDynamics(dt)
            rover.update(dt)
            t = system.GetChTime()
            if t + 1e-12 >= next_log:
                log.log(t)
                next_log += LOG_DT
    return read_csv(csv_path)


def window(data, key, t_from, t_to=None):
    ts = data["t_s"]
    return [v for t, v in zip(ts, data[key])
            if t >= t_from and (t_to is None or t <= t_to)]


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def unwrap_deg(series):
    """±180° 래핑된 각도 시계열을 연속 각도로 펼친다 (총 회전각 계산용)."""
    total = series[0]
    out = [total]
    for prev, cur in zip(series, series[1:]):
        d = cur - prev
        if d > 180.0:
            d -= 360.0
        elif d < -180.0:
            d += 360.0
        total += d
        out.append(total)
    return out


def mean_tractive_torque(data, names, t_from):
    """휠당 평균 견인 토크 (부호 유지).

    이상적 속도 모터 + NSC 접촉에서는 순간 토크에 큰 진동 노이즈가 있어
    |τ| 평균은 노이즈 진폭을 재게 된다. 물리적으로 의미 있는 '토크 요구량'은
    부호 있는 시간 평균 (등판 시 ≈ m·g·sinθ·r / N 과 비교 가능).
    """
    per_wheel = [mean(window(data, f"torque_{n}_nm", t_from)) for n in names]
    return sum(per_wheel) / len(per_wheel)


def peak_abs_torque(data, names, t_from):
    return max(
        max(abs(v) for v in window(data, f"torque_{n}_nm", t_from))
        for n in names
    )


class Checker:
    def __init__(self):
        self.results: list[dict] = []

    def check(self, name: str, ok: bool, detail: str):
        self.results.append({"name": name, "pass": bool(ok), "detail": detail})
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    @property
    def all_pass(self):
        return all(r["pass"] for r in self.results)


def verify_rover(spec_path: Path, slope_deg: float, checker: Checker) -> dict:
    spec, cfg = load_spec(spec_path)
    out_dir = OUT_ROOT / spec.rover_id
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"\n=== {spec.rover_id} (mass {spec.mass_kg} kg, "
          f"r {spec.wheel_radius_m} m, {spec.command_type}) ===")

    # ── A. 스폰/안착 ─────────────────────────────────────────────────────
    settle = simulate(spec, cfg, 0.0, mode="settle", duration=SETTLE_T,
                      csv_path=out_dir / "settle.csv")
    z_tail = window(settle, "z_m", SETTLE_T - 0.5)
    z_expect = spec.cg_xyz_m[2]
    z_err = abs(mean(z_tail) - z_expect)
    z_span = max(z_tail) - min(z_tail)
    pitch_tail = mean([abs(v) for v in window(settle, "pitch_deg", SETTLE_T - 0.5)])
    roll_tail = mean([abs(v) for v in window(settle, "roll_deg", SETTLE_T - 0.5)])

    checker.check(f"{spec.rover_id}/spawn CG 높이 유지",
                  z_err < 0.005,
                  f"|z−cg_z| = {z_err * 1000:.2f} mm (< 5 mm)")
    checker.check(f"{spec.rover_id}/spawn 튐·침하 없음",
                  z_span < 0.002,
                  f"마지막 0.5 s z 변동폭 = {z_span * 1000:.3f} mm (< 2 mm)")
    checker.check(f"{spec.rover_id}/spawn 자세 수평",
                  pitch_tail < 0.5 and roll_tail < 0.5,
                  f"|pitch| = {pitch_tail:.3f}°, |roll| = {roll_tail:.3f}° (< 0.5°)")

    # ── B. 평지 정속 주행 ────────────────────────────────────────────────
    flat = simulate(spec, cfg, 0.0, mode="cruise", duration=CRUISE_T,
                    csv_path=out_dir / "cruise_flat.csv", x_start=-2.0)
    t_end = SETTLE_T + CRUISE_T
    v_cmd = cruise_command(spec) * spec.wheel_radius_m \
        if spec.command_type == "wheel_speed" else None
    v_tail = mean(window(flat, "v_forward_mps", t_end - 2.0))
    slip_tail = mean([
        mean([abs(v) for v in window(flat, f"slip_{n}", t_end - 2.0)])
        for n in ("FL", "FR", "RL", "RR")
    ])
    y_drift = abs(window(flat, "y_m", 0)[-1])
    yaw_drift = abs(window(flat, "yaw_deg", 0)[-1])

    if v_cmd is not None:
        checker.check(f"{spec.rover_id}/cruise 상용 속도 도달",
                      abs(v_tail - v_cmd) < 0.1 * v_cmd,
                      f"v = {v_tail:.3f} m/s (목표 {v_cmd:.3f} ±10 %)")
    else:
        checker.check(f"{spec.rover_id}/cruise 전진",
                      v_tail > 0.05, f"v = {v_tail:.3f} m/s (> 0.05)")
    checker.check(f"{spec.rover_id}/cruise 슬립률 ≈ 0",
                  slip_tail < 0.05, f"mean|slip| = {slip_tail:.4f} (< 0.05)")
    checker.check(f"{spec.rover_id}/cruise 직진 유지",
                  y_drift < 0.05 and yaw_drift < 3.0,
                  f"|y| = {y_drift * 100:.2f} cm (< 5), |yaw| = {yaw_drift:.2f}° (< 3)")

    # ── C. 경사 등판 ─────────────────────────────────────────────────────
    slope = simulate(spec, cfg, slope_deg, mode="cruise", duration=CRUISE_T,
                     csv_path=out_dir / "cruise_slope.csv", x_start=-1.5)
    driven = ("FL", "FR", "RL", "RR")[:4]
    tau_flat = mean_tractive_torque(flat, driven, t_end - 2.0)
    tau_slope = mean_tractive_torque(slope, driven, t_end - 2.0)
    tau_peak = peak_abs_torque(slope, driven, t_end - 2.0)
    tau_expected = (spec.mass_kg * cfg.gravity_mps2
                    * math.sin(math.radians(slope_deg))
                    * spec.wheel_radius_m / 4.0)
    extra = tau_slope - tau_flat
    checker.check(f"{spec.rover_id}/slope 토크 요구 증가",
                  tau_slope > abs(tau_flat) + 0.5 * tau_expected,
                  f"τ_slope = {tau_slope * 1000:.2f} mN·m vs τ_flat = "
                  f"{tau_flat * 1000:.2f} mN·m (등판 기대 증가분 "
                  f"{tau_expected * 1000:.2f} mN·m, 실측 {extra * 1000:.2f})")
    checker.check(f"{spec.rover_id}/slope 등판 토크 물리 근사 일치",
                  0.5 * tau_expected < extra < 2.0 * tau_expected,
                  f"실측/기대 = {extra / tau_expected:.2f} (0.5–2.0 허용)")
    checker.check(f"{spec.rover_id}/slope 토크 한계 내",
                  tau_slope < spec.max_wheel_torque_nm,
                  f"평균 τ_slope = {tau_slope * 1000:.2f} mN·m < "
                  f"max {spec.max_wheel_torque_nm * 1000:.0f} mN·m "
                  f"(순간 피크 {tau_peak * 1000:.1f} mN·m — 속도모터 노이즈 포함)")

    # ── E. 제자리 선회 (skid steer) ──────────────────────────────────────
    # 좌우 바퀴 역회전 명령 → 제자리 좌선회. 대회 임무(경로 주행·정찰 지점
    # 접근)에 필수인 방향 전환 기능 확인. wheel_speed 명령 기준.
    yaw_gain = float("nan")
    if spec.command_type == "wheel_speed":
        pivot = simulate(spec, cfg, 0.0, mode="pivot", duration=4.0,
                         csv_path=out_dir / "pivot_turn.csv")
        yaw_series = unwrap_deg(window(pivot, "yaw_deg", 0))
        yaw_gain = yaw_series[-1] - yaw_series[0]
        px = abs(window(pivot, "x_m", 0)[-1])
        py = abs(window(pivot, "y_m", 0)[-1])
        checker.check(f"{spec.rover_id}/pivot 제자리 좌선회",
                      yaw_gain > 30.0,
                      f"4 s 선회각 = {yaw_gain:.1f}° (> 30°, 명령 부호와 방향 일치)")
        checker.check(f"{spec.rover_id}/pivot 위치 이탈 없음",
                      px < 0.1 and py < 0.1,
                      f"|x| = {px * 100:.1f} cm, |y| = {py * 100:.1f} cm (< 10 cm)")

    # ── F. 수납부피 (공고문: 300×300×200 mm) ─────────────────────────────
    # 마스트는 수납 시 접는 것으로 가정하고 제외. 대회 출품은 scout 이므로
    # main(연구용 플랫폼)은 참고 정보로만 보고한다.
    dims = overall_dimensions(spec, cfg)
    fits = dims[0] <= 0.30 and dims[1] <= 0.30 and dims[2] <= 0.20
    dims_txt = (f"{dims[0] * 1000:.0f}×{dims[1] * 1000:.0f}"
                f"×{dims[2] * 1000:.0f} mm (한도 300×300×200, 마스트 접이 제외)")
    if spec.rover_id.startswith("scout"):
        checker.check(f"{spec.rover_id}/수납부피 이내", fits, dims_txt)
    else:
        print(f"  [INFO] {spec.rover_id}/수납부피 (연구용, 제약 비적용): {dims_txt}")

    return {
        "spec": spec, "cfg": cfg, "out_dir": out_dir,
        "z_err_mm": z_err * 1000, "v_tail": v_tail, "slip": slip_tail,
        "tau_flat": tau_flat, "tau_slope": tau_slope,
        "tau_expected": tau_expected,
        "yaw_gain_deg": yaw_gain,
        "overall_dims_mm": [round(d * 1000, 1) for d in dims],
        "energy_slope_j": window(slope, "energy_j", 0)[-1],
        "energy_flat_j": window(flat, "energy_j", 0)[-1],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="로버 자체 검증 (headless)")
    ap.add_argument("--spec", type=Path, action="append",
                    help="스펙 yaml (여러 번 지정 가능). 기본: scout+main")
    ap.add_argument("--slope", type=float, default=10.0, help="경사 [deg]")
    ap.add_argument("--no-plot", action="store_true", help="PNG 생성 생략")
    args = ap.parse_args()

    spec_paths = args.spec or DEFAULT_SPECS
    checker = Checker()
    results = [verify_rover(p, args.slope, checker) for p in spec_paths]

    # ── D. 스펙 간 비교 (질량·휠 차이 반영 확인) ─────────────────────────
    if len(results) >= 2:
        a, b = results[0], results[1]
        light, heavy = (a, b) if a["spec"].mass_kg < b["spec"].mass_kg else (b, a)
        print(f"\n=== 비교: {light['spec'].rover_id} vs {heavy['spec'].rover_id} ===")
        checker.check("compare/무거운 로버가 등판 토크 더 큼",
                      heavy["tau_slope"] > light["tau_slope"],
                      f"{heavy['spec'].rover_id} {heavy['tau_slope'] * 1000:.2f} "
                      f"> {light['spec'].rover_id} {light['tau_slope'] * 1000:.2f} mN·m")
        checker.check("compare/무거운 로버가 등판 에너지 더 씀",
                      heavy["energy_slope_j"] > light["energy_slope_j"],
                      f"{heavy['spec'].rover_id} {heavy['energy_slope_j']:.3f} J "
                      f"> {light['spec'].rover_id} {light['energy_slope_j']:.3f} J")
        ratio_tau = heavy["tau_slope"] / max(light["tau_slope"], 1e-9)
        ratio_expect = (heavy["tau_expected"] / light["tau_expected"])
        checker.check("compare/토크비가 질량·휠반경비 근사",
                      0.6 * ratio_expect < ratio_tau < 1.6 * ratio_expect,
                      f"실측비 {ratio_tau:.2f} vs 기대비(m·r) {ratio_expect:.2f}")

    # ── 요약 저장 ────────────────────────────────────────────────────────
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    summary = {
        "slope_deg": args.slope,
        "checks": checker.results,
        "rovers": {
            r["spec"].rover_id: {
                "mass_kg": r["spec"].mass_kg,
                "overall_dims_mm": r["overall_dims_mm"],
                "settle_z_err_mm": round(r["z_err_mm"], 3),
                "cruise_v_mps": round(r["v_tail"], 4),
                "cruise_mean_abs_slip": round(r["slip"], 5),
                "pivot_yaw_4s_deg": round(r["yaw_gain_deg"], 1),
                "tau_flat_mnm": round(r["tau_flat"] * 1000, 3),
                "tau_slope_mnm": round(r["tau_slope"] * 1000, 3),
                "tau_slope_expected_extra_mnm":
                    round(r["tau_expected"] * 1000, 3),
                "energy_flat_j": round(r["energy_flat_j"], 4),
                "energy_slope_j": round(r["energy_slope_j"], 4),
            } for r in results
        },
    }
    (OUT_ROOT / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nsummary: {OUT_ROOT / 'summary.json'}")

    if not args.no_plot:
        from plot_results import plot_all
        plot_all(OUT_ROOT, [r["spec"].rover_id for r in results])

    n_fail = sum(1 for r in checker.results if not r["pass"])
    print(f"\n{'모든 체크 통과' if checker.all_pass else f'{n_fail}개 체크 실패'} "
          f"({len(checker.results)}개 중)")
    return 0 if checker.all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
