from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.chrono.rover_factory import build_rover_from_spec
from src.chrono.system_factory import make_nsc_system
from src.chrono.terrain_factory import build_terrain_from_scenario
from src.registries import RoverRegistry, TerrainMaterialRegistry, TerrainRegistry


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


def _load_rover(rover_key: str):
    rover_ids = {"scout": "scout_v01", "main": "main_v01"}
    if rover_key not in rover_ids:
        raise ValueError(f"unknown rover_key {rover_key!r}")
    rover = RoverRegistry(PROJECT_ROOT / "rover_models", repo_root=PROJECT_ROOT).load(rover_ids[rover_key])
    metadata = dict(rover.metadata)
    metadata["command_type"] = "wheel_speed"
    return replace(rover, metadata=metadata)


def _load_terrain(terrain_id: str):
    terrain = TerrainRegistry(PROJECT_ROOT / "terrain_scenarios", repo_root=PROJECT_ROOT).load(terrain_id)
    material = TerrainMaterialRegistry(PROJECT_ROOT / "terrain_materials", repo_root=PROJECT_ROOT).load(terrain.material_id)
    return terrain, material


def run_viewer(args: argparse.Namespace) -> int:
    import pychrono as chrono  # type: ignore
    import pychrono.irrlicht as chronoirr  # type: ignore

    terrain, material = _load_terrain(args.terrain)
    rover_spec = _load_rover(args.rover)
    system = make_nsc_system(gravity_mps2=(0.0, 0.0, -9.81))
    terrain_artifact = build_terrain_from_scenario(system, terrain, material)

    spawn_x = -2.50
    spawn_y = 0.0
    spawn_z = max(0.08, rover_spec.wheel_radius_m + 0.03)
    spawn_frame = chrono.ChFramed(chrono.ChVector3d(spawn_x, spawn_y, spawn_z), chrono.QUNIT)
    color = (0.12, 0.38, 0.82) if args.rover == "main" else (0.20, 0.62, 0.35)
    rover = build_rover_from_spec(system, rover_spec, spawn_frame=spawn_frame, color=color)

    max_speed = float(rover_spec.metadata.get("max_wheel_speed_radps", 10.0))
    target_speed_radps = max_speed * max(0.0, min(args.command_fraction, 1.0))

    title = f"Jongmin Arena Viewer - {args.rover} / {terrain.terrain_id}"
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    _set_visual_defaults(vis, chrono, args.width, args.height, title)

    camera_pos = chrono.ChVector3d(-2.4, -2.1, 1.05)
    camera_target = chrono.ChVector3d(-1.6, 0.0, 0.12)
    if hasattr(vis, "AddCamera"):
        vis.AddCamera(camera_pos, camera_target)

    timer = chrono.ChRealtimeStepTimer() if hasattr(chrono, "ChRealtimeStepTimer") else None
    next_log = 0.0
    min_z = float(rover.chassis.GetPos().z)
    max_contact_count = 0

    print(
        json.dumps(
            {
                "event": "viewer_started",
                "viewer": "jongmin_arena",
                "rover": args.rover,
                "rover_id": rover_spec.rover_id,
                "terrain_id": terrain.terrain_id,
                "target_speed_radps": target_speed_radps,
                "terrain_artifact_keys": sorted(terrain_artifact.keys()) if isinstance(terrain_artifact, dict) else [],
                "note": "z_m is chassis/body center position, not wheel-ground contact height.",
            },
            ensure_ascii=False,
        ),
        flush=True,
    )

    while vis.Run() and system.GetChTime() < args.duration:
        t = float(system.GetChTime())
        ramp = min(1.0, t / max(args.ramp, 1e-9))
        rover.set_command(target_speed_radps * ramp)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        system.DoStepDynamics(args.step_size)
        rover.update(args.step_size)

        pos = rover.chassis.GetPos()
        vel = rover.chassis.GetPosDt()
        roll, pitch, yaw = rover.rpy_rad()
        contacts = _contact_count(system)
        min_z = min(min_z, float(pos.z))
        max_contact_count = max(max_contact_count, contacts)

        t = float(system.GetChTime())
        if t + 1e-12 >= next_log:
            print(
                json.dumps(
                    {
                        "event": "telemetry",
                        "t_s": round(t, 4),
                        "x_m": round(float(pos.x), 5),
                        "y_m": round(float(pos.y), 5),
                        "z_m": round(float(pos.z), 5),
                        "vz_mps": round(float(vel.z), 5),
                        "contact_count": contacts,
                        "max_contact_count": max_contact_count,
                        "roll_deg": round(math.degrees(roll), 3),
                        "pitch_deg": round(math.degrees(pitch), 3),
                        "yaw_deg": round(math.degrees(yaw), 3),
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )
            next_log += args.log_period

        if timer is not None:
            timer.Spin(args.step_size)

    final_pos = rover.chassis.GetPos()
    print(
        json.dumps(
            {
                "event": "viewer_finished",
                "status": "completed",
                "final_chassis_xyz_m": _vec3_to_tuple(final_pos),
                "min_z_m": min_z,
                "max_contact_count": max_contact_count,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Open an Irrlicht window for Jongmin's arena terrain.")
    parser.add_argument("--rover", choices=("scout", "main", "scout_then_main"), default="scout_then_main")
    parser.add_argument("--terrain", default="jongmin_arena_v01")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--ramp", type=float, default=0.4)
    parser.add_argument("--step-size", type=float, default=0.001)
    parser.add_argument("--log-period", type=float, default=0.25)
    parser.add_argument("--command-fraction", type=float, default=0.4)
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