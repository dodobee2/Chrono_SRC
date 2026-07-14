from __future__ import annotations

import argparse

from .smoke_scenario import (
    SmokeScenarioConfig,
    _make_box,
    _make_floor,
    _make_system,
    _set_gravity,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Open an Irrlicht window for the PyChrono box-drop smoke scenario.")
    parser.add_argument("--duration", type=float, default=10.0, help="Maximum viewer runtime in simulation seconds.")
    parser.add_argument("--step-size", type=float, default=0.001, help="Chrono dynamics step size in seconds.")
    parser.add_argument("--width", type=int, default=1280, help="Window width in pixels.")
    parser.add_argument("--height", type=int, default=720, help="Window height in pixels.")
    args = parser.parse_args()

    import pychrono as chrono  # type: ignore
    import pychrono.irrlicht as chronoirr  # type: ignore

    config = SmokeScenarioConfig(duration_s=args.duration, step_size_s=args.step_size)
    system = _make_system(chrono)
    _set_gravity(system, chrono, config.gravity_mps2)
    floor = _make_floor(system, chrono)
    box = _make_box(system, chrono, config)
    system.Add(floor)
    system.Add(box)

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    if hasattr(chrono, "CameraVerticalDir_Z") and hasattr(vis, "SetCameraVertical"):
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(args.width, args.height)
    vis.SetWindowTitle("PyChrono Smoke Viewer - Box Drop")
    vis.Initialize()
    if hasattr(vis, "AddSkyBox"):
        vis.AddSkyBox()
    if hasattr(vis, "AddCamera"):
        vis.AddCamera(chrono.ChVector3d(1.4, -1.8, 1.0), chrono.ChVector3d(0.0, 0.0, 0.15))
    if hasattr(vis, "AddTypicalLights"):
        vis.AddTypicalLights()

    timer = chrono.ChRealtimeStepTimer() if hasattr(chrono, "ChRealtimeStepTimer") else None
    while vis.Run() and system.GetChTime() < args.duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(args.step_size)
        if timer is not None:
            timer.Spin(args.step_size)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
