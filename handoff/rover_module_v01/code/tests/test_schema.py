"""RoverSpec yaml 로더/검증 테스트."""

import dataclasses
from pathlib import Path

import pytest

from rover_schema import (
    RoverSpec,
    SimConfig,
    SpecValidationError,
    load_spec,
    validate_spec,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SPEC_DIR = PROJECT_ROOT / "specs" / "rovers"


def valid_kwargs(**over):
    kw = dict(
        rover_id="test", mass_kg=1.0, cg_xyz_m=(0.0, 0.0, 0.05),
        wheel_count=4, driven_wheel_count=4,
        wheel_radius_m=0.03, wheel_width_m=0.025,
        wheelbase_m=0.16, track_width_m=0.14, ground_clearance_m=0.03,
        command_type="wheel_speed",
        max_wheel_torque_nm=0.15, max_wheel_speed_radps=20.0,
    )
    kw.update(over)
    return kw


def test_load_scout_and_main():
    for name in ("scout_v01.yaml", "main_v01.yaml"):
        spec, cfg = load_spec(SPEC_DIR / name)
        assert isinstance(spec, RoverSpec)
        assert isinstance(cfg, SimConfig)
        assert spec.mass_kg > 0
        assert spec.wheel_count == 4


def test_scout_matches_initial_estimates():
    spec, _ = load_spec(SPEC_DIR / "scout_v01.yaml")
    assert spec.mass_kg == 1.8
    assert spec.wheel_radius_m == 0.03
    assert spec.wheelbase_m == 0.16
    assert spec.command_type == "wheel_speed"
    assert spec.driven_wheel_count == 4


def test_mass_within_team_policy():
    """팀 결정(2026-07-14): scout·main 모두 1.5~3 kg 범위."""
    for name in ("scout_v01.yaml", "main_v01.yaml"):
        spec, _ = load_spec(SPEC_DIR / name)
        assert 1.5 <= spec.mass_kg <= 3.0, f"{spec.rover_id}: {spec.mass_kg} kg"


@pytest.mark.parametrize("bad", [
    dict(mass_kg=-1.0),
    dict(mass_kg=0.0),
    dict(wheel_count=3),
    dict(driven_wheel_count=0),
    dict(driven_wheel_count=5),
    dict(driven_wheel_count=3),
    dict(command_type="velocity"),
    dict(max_wheel_torque_nm=0.0),
    dict(max_wheel_speed_radps=-1.0),
    dict(cg_xyz_m=(0.0, 0.0, 0.01)),        # CG < 지상고
    dict(ground_clearance_m=0.5),           # 지상고 > 휠 직경
])
def test_invalid_specs_rejected(bad):
    with pytest.raises(SpecValidationError):
        validate_spec(RoverSpec(**valid_kwargs(**bad)))


def test_missing_field_rejected(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("rover:\n  rover_id: x\n  mass_kg: 1.0\n", encoding="utf-8")
    with pytest.raises(SpecValidationError, match="누락"):
        load_spec(p)


def test_unknown_field_rejected(tmp_path):
    import yaml
    d = {"rover": {**valid_kwargs(), "cg_xyz_m": [0, 0, 0.05], "extra": 1}}
    p = tmp_path / "bad.yaml"
    p.write_text(yaml.safe_dump(d), encoding="utf-8")
    with pytest.raises(SpecValidationError, match="알 수 없는"):
        load_spec(p)


def test_contract_fields_unchanged():
    """팀장 데이터 계약과 필드명/순서가 일치하는지 고정."""
    names = [f.name for f in dataclasses.fields(RoverSpec)]
    assert names == [
        "rover_id", "mass_kg", "cg_xyz_m",
        "wheel_count", "driven_wheel_count",
        "wheel_radius_m", "wheel_width_m",
        "wheelbase_m", "track_width_m", "ground_clearance_m",
        "command_type", "max_wheel_torque_nm", "max_wheel_speed_radps",
    ]
