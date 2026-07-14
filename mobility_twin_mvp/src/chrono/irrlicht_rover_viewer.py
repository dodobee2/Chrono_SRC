from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.experiments.rigid_transfer_pilot.presets import CONDITIONS, DEFAULT_COMMAND, RigidCondition, TorqueCommand
from src.experiments.rigid_transfer_pilot.scenario import build_pilot_scenario


def _condition_by_id(condition_id: str) -> RigidCondition:
    for condition in CONDITIONS:
        if condition.condition_id == condition_id:
            return condition
    valid = ", ".join(condition.condition_id for condition in CONDITIONS)
    raise ValueError(f"unknown condition {condition_id!r}; expected one of: {valid}")


def _contact_count(system) -> int:
    if hasattr(system, "GetNumContacts") and callable(system.GetNumContacts):
        return int(system.GetNumContacts())
    if hasattr(system, "GetContactContainer") and callable(system.GetContactContainer):
        container = system.GetContactContainer()
        if hasattr(container, "GetNumContacts") and callable(container.GetNumContacts):
            return int(container.GetNumContacts())
    return 0


def _vec3_to_tuple(vec) -> tuple[float, float, float]:
    return (float(vec.x), float(vec.y), float(vec.z))


def _set_visual_defaults(vis, chrono, width: int, height: int, title: str) -> None:
    vis.SetWindowSize(width, height)
    vis.SetWindowTitle(title)
    vis.Initialize()
    if hasattr(chrono, "CameraVerticalDir_Z") and hasattr(vis, "SetCameraVertical"):
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    if hasattr(vis, "AddSkyBox"):
        vis.AddSkyBox()
    if hasattr(vis, "AddTypicalLights"):
        vis.AddTypicalLights()


def run_viewer(args: argparse.Namespace) -> int:
    import pychrono as chrono  # type: ignore
    import pychrono.irrlicht as chronoirr  # type: ignore

    condition = _condition_by_id(args.condition)
    command = TorqueCommand(
        torque_fraction=args.torque_fraction,
        duration_s=args.duration,
        settle_s=args.settle,
        ramp_s=args.ramp,
        timestep_s=args.step_size,
        log_period_s=args.log_period,
    )

    scn = build_pilot_scenario(args.rover, condition, command.torque_fraction)
    rover = scn.rover
    target_torque_nm = command.torque_fraction * scn.rover_spec.max_wheel_torque_nm

    title = f"Rigid Transfer Rover Viewer - {args.rover} / {condition.condition_id}"
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(scn.system)
    _set_visual_defaults(vis, chrono, args.width, args.height, title)

    start_pos = rover.chassis.GetPos()
    camera_pos = chrono.ChVector3d(float(start_pos.x) + 0.9, -1.5, 0.65)
    camera_target = chrono.ChVector3d(float(start_pos.x) + 0.35, 0.0, 0.08)
    if hasattr(vis, "AddCamera"):
        vis.AddCamera(camera_pos, camera_target)

    timer = chrono.ChRealtimeStepTimer() if hasattr(chrono, "ChRealtimeStepTimer") else None
    total_duration = command.settle_s + command.duration_s
    next_log = 0.0
    min_z = float(start_pos.z)
    max_contact_count = 0

    print(
        json.dumps(
            {
                "event": "viewer_started",
                "rover": args.rover,
                "rover_id": scn.rover_spec.rover_id,
                "condition": condition.condition_id,
                "slope_deg": condition.slope_deg,
                "friction_key": condition.friction_key,
                "obstacle_key": condition.obstacle_key,
                "initial_chassis_xyz_m": _vec3_to_tuple(start_pos),
                "note": "z_m is chassis/body center position, not wheel-ground contact height.",
            },
            ensure_ascii=False,
        ),
        flush=True,
    )

    while vis.Run() and scn.system.GetChTime() < total_duration:
        t = float(scn.system.GetChTime())
        if t >= command.settle_s:
            ramp = min(1.0, (t - command.settle_s) / max(command.ramp_s, 1e-9))
            rover.set_command(target_torque_nm * ramp)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        scn.system.DoStepDynamics(command.timestep_s)
        rover.update(command.timestep_s)

        pos = rover.chassis.GetPos()
        vel = rover.chassis.GetPosDt()
        roll, pitch, yaw = rover.rpy_rad()
        contacts = _contact_count(scn.system)
        min_z = min(min_z, float(pos.z))
        max_contact_count = max(max_contact_count, contacts)

        t = float(scn.system.GetChTime())
        if t + 1e-12 >= next_log:
            print(
                json.dumps(
                    {
                        "event": "telemetry",
                        "t_s": round(t, 4),
                        "x_m": round(float(pos.x), 5),
                        "y_m": round(float(pos.y), 5),
                        "z_m": round(float(pos.z), 5),
                        "min_z_m": round(min_z, 5),
                        "vz_mps": round(float(vel.z), 5),
                        "contact_count": contacts,
                        "max_contact_count": max_contact_count,
                        "roll_deg": round(math.degrees(roll), 3),
                        "pitch_deg": round(math.degrees(pitch), 3),
                        "yaw_deg": round(math.degrees(yaw), 3),
                    }
                ),
                flush=True,
            )
            next_log += command.log_period_s

        if timer is not None:
            timer.Spin(command.timestep_s)

    final_pos = rover.chassis.GetPos()
    final_vel = rover.chassis.GetPosDt()
    print(
        json.dumps(
            {
                "event": "viewer_finished",
                "status": "completed",
                "final_chassis_xyz_m": _vec3_to_tuple(final_pos),
                "final_vz_mps": float(final_vel.z),
                "min_z_m": min_z,
                "max_contact_count": max_contact_count,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Open an Irrlicht window for the rigid-transfer rover pilot.")
    parser.add_argument("--rover", choices=("scout", "main", "scout_then_main"), default="main")
    parser.add_argument(
        "--condition",
        choices=tuple(condition.condition_id for condition in CONDITIONS),
        default="flat",
    )
    parser.add_argument("--duration", type=float, default=6.0, help="Command duration after settle time, in seconds.")
    parser.add_argument("--settle", type=float, default=DEFAULT_COMMAND.settle_s)
    parser.add_argument("--ramp", type=float, default=DEFAULT_COMMAND.ramp_s)
    parser.add_argument("--step-size", type=float, default=DEFAULT_COMMAND.timestep_s)
    parser.add_argument("--log-period", type=float, default=0.25)
    parser.add_argument("--torque-fraction", type=float, default=DEFAULT_COMMAND.torque_fraction)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()
    if args.rover == "scout_then_main":
        for rover_key in ("scout", "main"):
            phase_args = argparse.Namespace(**vars(args))
            phase_args.rover = rover_key
            code = run_viewer(phase_args)
            if code != 0:
                return code
        return 0
    return run_viewer(args)


if __name__ == "__main__":
    raise SystemExit(main())
