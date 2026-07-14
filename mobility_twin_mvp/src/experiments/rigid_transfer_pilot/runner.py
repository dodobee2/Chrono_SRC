"""Drives one headless torque-limited straight-cruise rigid pilot run.

Only needs pychrono core (ChSystemNSC, ChBodyEasyBox, Bullet collision) --
no pychrono.vehicle import anywhere in this pilot, so it runs today even
though src/experiments/scm_pilot is blocked (see docs/ENVIRONMENT_SETUP.md).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .metrics import RunSummary, summarize
from .presets import DEFAULT_COMMAND, RigidCondition, TorqueCommand
from .scenario import build_pilot_scenario


@dataclass(frozen=True)
class PilotRunResult:
    trajectory: list[dict[str, float]]
    summary: RunSummary


def _contact_count(system) -> int:
    """Mirrors src/chrono/smoke_scenario.py::_make_contact_counter's API introspection."""
    if hasattr(system, "GetNumContacts") and callable(system.GetNumContacts):
        return int(system.GetNumContacts())
    if hasattr(system, "GetContactContainer") and callable(system.GetContactContainer):
        container = system.GetContactContainer()
        if hasattr(container, "GetNumContacts") and callable(container.GetNumContacts):
            return int(container.GetNumContacts())
    return 0


def run_pilot(
    rover_key: str,
    condition: RigidCondition,
    command: TorqueCommand = DEFAULT_COMMAND,
) -> PilotRunResult:
    """Run one headless torque-limited straight-cruise rigid scenario."""
    started = time.perf_counter()
    scn = build_pilot_scenario(rover_key, condition, command.torque_fraction)
    rover = scn.rover
    target_torque_nm = command.torque_fraction * scn.rover_spec.max_wheel_torque_nm

    trajectory: list[dict[str, float]] = []
    t = 0.0
    next_log = 0.0
    total_duration = command.settle_s + command.duration_s

    while t < total_duration:
        if t >= command.settle_s:
            ramp = min(1.0, (t - command.settle_s) / max(command.ramp_s, 1e-9))
            rover.set_command(target_torque_nm * ramp)
        scn.system.DoStepDynamics(command.timestep_s)
        rover.update(command.timestep_s)
        t = scn.system.GetChTime()

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
                    "contact_count": _contact_count(scn.system),
                }
            )
            next_log += command.log_period_s

    wall_time_s = time.perf_counter() - started
    summary = summarize(
        trajectory,
        rover_id=scn.rover_spec.rover_id,
        condition_id=condition.condition_id,
        wall_time_s=wall_time_s,
    )
    return PilotRunResult(trajectory=trajectory, summary=summary)
