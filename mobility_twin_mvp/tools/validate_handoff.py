from __future__ import annotations

import argparse
import sys
import warnings
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.registries import (  # noqa: E402
    ContactPairRegistry,
    ControlProfileRegistry,
    ObservationRegistry,
    RoverRegistry,
    TerrainMaterialRegistry,
    TerrainRegistry,
)


def _print_ids(label: str, ids: list[str]) -> None:
    print(f"{label}: {len(ids)}")
    for item in ids:
        print(f"  - {item}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate mobility_twin_mvp handoff YAML registries.")
    parser.add_argument("--rover", default=None, help="RoverSpec id to validate")
    parser.add_argument("--terrain", default=None, help="TerrainScenario id to validate")
    parser.add_argument("--control", default=None, help="ControlProfile id to validate")
    parser.add_argument("--observation", default=None, help="ScoutObservation id to validate")
    parser.add_argument("--contact-pair", default=None, help="ContactPairSpec id to validate")
    args = parser.parse_args()

    rover_registry = RoverRegistry(PROJECT_ROOT / "rover_models", repo_root=PROJECT_ROOT)
    terrain_registry = TerrainRegistry(PROJECT_ROOT / "terrain_scenarios", repo_root=PROJECT_ROOT)
    control_registry = ControlProfileRegistry(PROJECT_ROOT / "control_profiles", repo_root=PROJECT_ROOT)
    observation_registry = ObservationRegistry(PROJECT_ROOT / "observations", repo_root=PROJECT_ROOT)
    material_registry = TerrainMaterialRegistry(PROJECT_ROOT / "terrain_materials", repo_root=PROJECT_ROOT)
    contact_registry = ContactPairRegistry(PROJECT_ROOT / "contact_pairs", repo_root=PROJECT_ROOT)

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        rover_ids = rover_registry.ids()
        terrain_ids = terrain_registry.ids()
        control_ids = control_registry.ids()
        observation_ids = observation_registry.ids()
        material_ids = material_registry.ids()
        contact_ids = contact_registry.ids()

        rover_registry.load_all()
        terrain_registry.load_all()
        control_registry.load_all()
        observation_registry.load_all()
        material_registry.load_all()
        contact_registry.load_all()

        _print_ids("Rovers", rover_ids)
        _print_ids("Terrains", terrain_ids)
        _print_ids("Controls", control_ids)
        _print_ids("Observations", observation_ids)
        _print_ids("Materials", material_ids)
        _print_ids("ContactPairs", contact_ids)

        if args.rover:
            rover = rover_registry.load(args.rover)
            print(f"selected rover OK: {rover.rover_id}")
        else:
            rover = None
        if args.terrain:
            terrain = terrain_registry.load(args.terrain)
            print(f"selected terrain OK: {terrain.terrain_id}")
            if terrain.material_id not in material_ids:
                raise ValueError(f"terrain material_id is not registered: {terrain.material_id}")
            print(f"terrain material OK: {terrain.material_id}")
        else:
            terrain = None
        if args.control:
            control = control_registry.load(args.control)
            print(f"selected control OK: {control.profile_id}")
        else:
            control = None
        if args.observation:
            observation = observation_registry.load(args.observation)
            print(f"selected observation OK: {observation.observation_id}")
            if terrain and observation.terrain_id != terrain.terrain_id:
                raise ValueError("observation terrain_id does not match selected terrain")
            if control and observation.control_profile_id != control.profile_id:
                raise ValueError("observation control_profile_id does not match selected control")
        if args.contact_pair:
            if rover is None or terrain is None:
                raise ValueError("--contact-pair validation also needs --rover and --terrain")
            pair = contact_registry.load(args.contact_pair)
            contact_registry.validate_references(pair, rover, terrain, material_registry)
            print(f"selected contact pair OK: {pair.contact_pair_id}")
        elif rover and terrain:
            matches = contact_registry.ids_for(rover.wheel_material_id, terrain.material_id)
            print("matching contact pairs:")
            for match in matches:
                print(f"  - {match}")
            if not matches and rover.fallback_mu_eff is None:
                raise ValueError("no matching contact pair and rover has no fallback contact values")

    if caught:
        print("warnings:")
        for item in caught:
            print(f"  - {item.message}")

    print("handoff validation completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())