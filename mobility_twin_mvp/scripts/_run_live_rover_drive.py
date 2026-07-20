"""Runs one (rover_id, terrain_id, control_id) real-Chrono drive in its own process.

Generalizes rigid_transfer_pilot's fixed scout_v01/main_v01 + preset
conditions to ANY registered RoverSpec/TerrainScenario/ControlProfile,
reusing the same validated building blocks (system_factory, rover_factory,
terrain_factory) with no special-case slope/obstacle handling of its own --
whatever terrain_factory.build_terrain_from_scenario does not support raises
straight through as an error instead of a silent approximation.

Not all registered content is drivable today, by design, not by oversight:
build_rigid_flat_terrain refuses any nonzero slope or any obstacle,
terrain_type "rocky"/"mixed" has no TerrainScenario-driven builder yet, SCM
terrain needs pychrono.vehicle which fails to import in this environment
(see docs/ENVIRONMENT_SETUP.md), and code_factory terrains (e.g.
jongmin_arena_v01) are unreviewed. The caller (app.py's "3-C" panel) is
expected to surface whatever error comes back verbatim, not hide it.

Internal helper for src/chrono/subprocess_isolation.py's per-attempt
isolation -- not meant to be run manually, though nothing stops you.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.chrono.rover_factory import build_rover_from_spec
from src.chrono.system_factory import make_nsc_system
from src.chrono.terrain_factory import build_terrain_from_scenario
from src.experiments.rigid_transfer_pilot.metrics import TRAJECTORY_COLUMNS, summarize
from src.registries import ControlProfileRegistry, RoverRegistry, TerrainMaterialRegistry, TerrainRegistry

SETTLE_S = 0.5
RAMP_S = 0.5
TIMESTEP_S = 2.0e-3
LOG_PERIOD_S = 0.05
SPAWN_DROP_M = 0.1


def _contact_count(system) -> int:
    if hasattr(system, "GetNumContacts") and callable(system.GetNumContacts):
        return int(system.GetNumContacts())
    if hasattr(system, "GetContactContainer") and callable(system.GetContactContainer):
        container = system.GetContactContainer()
        if hasattr(container, "GetNumContacts") and callable(container.GetNumContacts):
            return int(container.GetNumContacts())
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rover-id", required=True)
    parser.add_argument("--terrain-id", required=True)
    parser.add_argument("--control-id", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    rover_registry = RoverRegistry(PROJECT_ROOT / "rover_models", repo_root=PROJECT_ROOT)
    terrain_registry = TerrainRegistry(PROJECT_ROOT / "terrain_scenarios", repo_root=PROJECT_ROOT)
    control_registry = ControlProfileRegistry(PROJECT_ROOT / "control_profiles", repo_root=PROJECT_ROOT)
    material_registry = TerrainMaterialRegistry(PROJECT_ROOT / "terrain_materials", repo_root=PROJECT_ROOT)

    rover_spec = rover_registry.load(args.rover_id)
    terrain = terrain_registry.load(args.terrain_id)
    control = control_registry.load(args.control_id)
    material = material_registry.load(terrain.material_id) if terrain.material_id in material_registry.ids() else None

    import pychrono as chrono

    system = make_nsc_system()
    build_terrain_from_scenario(system, terrain, material)

    spawn_frame = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, SPAWN_DROP_M), chrono.QUNIT)
    rover = build_rover_from_spec(system, rover_spec, spawn_frame=spawn_frame)

    # throttle is a 0..1 fraction of whichever native command range this
    # rover is registered with -- wheel_speed and wheel_torque modes have
    # different units and RoverInstance.set_command interprets the raw value
    # differently depending on rover.spec.command_type (see rover_builder.py).
    if rover.spec.command_type == "wheel_speed":
        target_value = control.throttle * rover.spec.max_wheel_speed_radps
    else:
        target_value = control.throttle * rover.spec.max_wheel_torque_nm

    total_duration = SETTLE_S + control.duration_s
    trajectory: list[dict] = []
    t = 0.0
    next_log = 0.0
    while t < total_duration:
        if t >= SETTLE_S:
            ramp = min(1.0, (t - SETTLE_S) / RAMP_S)
            rover.set_command(target_value * ramp)
        system.DoStepDynamics(TIMESTEP_S)
        rover.update(TIMESTEP_S)
        t = system.GetChTime()

        if t + 1e-12 >= next_log:
            pos = rover.chassis.GetPos()
            roll, pitch, yaw = rover.rpy_rad()
            slips = rover.slip_ratios()
            torques = rover.wheel_torques()
            trajectory.append(
                {
                    "t_s": t,
                    "x_m": float(pos.x),
                    "y_m": float(pos.y),
                    "z_m": float(pos.z),
                    "roll_deg": math.degrees(roll),
                    "pitch_deg": math.degrees(pitch),
                    "yaw_deg": math.degrees(yaw),
                    "v_forward_mps": rover.forward_speed(),
                    "mean_slip": sum(slips.values()) / len(slips) if slips else 0.0,
                    "mean_wheel_torque_nm": sum(torques.values()) / len(torques) if torques else 0.0,
                    "power_w": rover.total_power_w(),
                    "energy_j": rover.energy_proxy_j(),
                    "contact_count": _contact_count(system),
                }
            )
            next_log += LOG_PERIOD_S

    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out / "trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=TRAJECTORY_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(trajectory)
    summary = summarize(trajectory, rover_id=rover_spec.rover_id, condition_id=terrain.terrain_id, wall_time_s=0.0)
    (args.out / "summary.json").write_text(json.dumps(summary.to_dict(), indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
