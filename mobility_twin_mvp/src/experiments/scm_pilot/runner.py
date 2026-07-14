"""Drives one headless straight-cruise pilot run and logs a trajectory.

UNVERIFIED end-to-end (see scenario.py docstring): requires pychrono.vehicle,
which currently fails to import in this project's `chrono` conda env.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .metrics import RunSummary, summarize
from .presets import DEFAULT_COMMAND, CruiseCommand
from .scenario import PilotScenario, build_pilot_scenario


@dataclass(frozen=True)
class PilotRunResult:
    trajectory: list[dict[str, float]]
    summary: RunSummary


def _cruise_command_value(scenario: PilotScenario, command: CruiseCommand) -> float:
    """Mirrors handoff/rover_module_v01/code/scripts/run_rover_check.py::cruise_command."""
    spec = scenario.rover.spec
    if spec.command_type == "wheel_speed":
        return command.wheel_speed_fraction * spec.max_wheel_speed_radps
    return command.wheel_torque_fraction * spec.max_wheel_torque_nm


def _wheel_sinkage_m(wheel_z: float, wheel_radius_m: float) -> float:
    """Geometric sinkage estimate: how far the wheel center has dropped below
    where it would rest on an undeformed rigid floor.

    The SCM patch's own mesh is never tilted (see scenario.py -- slope is
    applied by tilting gravity instead), so the nominal, undeformed ground
    plane is always z=0 regardless of slope_deg. A wheel resting on a rigid
    floor at z=0 has its center at z=wheel_radius_m; anything lower is
    interpreted as soil deformation. This avoids depending on an unconfirmed
    SCMTerrain deformation-query API (pychrono.vehicle does not import in
    this env, so no such API could be checked -- see module docstring).
    """
    return max(0.0, wheel_radius_m - wheel_z)


def run_pilot(
    rover_key: str,
    slope_deg: float,
    soil_key: str,
    command: CruiseCommand = DEFAULT_COMMAND,
) -> PilotRunResult:
    """Run one headless straight-cruise pilot scenario and return its trajectory + summary."""
    started = time.perf_counter()
    scn = build_pilot_scenario(rover_key, slope_deg, soil_key)
    rover = scn.rover
    wheel_radius_m = scn.rover_spec.wheel_radius_m
    target = _cruise_command_value(scn, command)

    trajectory: list[dict[str, float]] = []
    t = 0.0
    next_log = 0.0
    total_duration = command.settle_s + command.duration_s

    while t < total_duration:
        if t >= command.settle_s:
            ramp = min(1.0, (t - command.settle_s) / max(command.ramp_s, 1e-9))
            rover.set_command(target * ramp)
        scn.system.DoStepDynamics(command.timestep_s)
        rover.update(command.timestep_s)
        t = scn.system.GetChTime()

        if t + 1e-12 >= next_log:
            pos = rover.chassis.GetPos()
            roll, pitch, yaw = rover.rpy_rad()
            slips = rover.slip_ratios()
            torques = rover.wheel_torques()
            sinkages = [_wheel_sinkage_m(float(wheel.GetPos().z), wheel_radius_m) for wheel in rover.wheels.values()]
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
                    "mean_sinkage_m": (sum(sinkages) / len(sinkages)) if sinkages else None,
                    "max_sinkage_m": max(sinkages) if sinkages else None,
                }
            )
            next_log += command.log_period_s

    wall_time_s = time.perf_counter() - started
    summary = summarize(
        trajectory,
        rover_id=scn.rover_spec.rover_id,
        slope_deg=slope_deg,
        soil_material_id=scn.soil_material.material_id,
        wall_time_s=wall_time_s,
    )
    return PilotRunResult(trajectory=trajectory, summary=summary)
