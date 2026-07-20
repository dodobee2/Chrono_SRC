"""Adapter for Jongmin's handoff arena map.

This file intentionally stays thin: the handed-off implementation lives in
assets/map.py and remains replaceable by the terrain owner. The common terrain
factory loads build_terrain(system, terrain, material) through
TerrainScenario.geometry.factory_uri.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
import importlib.util
import sys
import types


def _load_arena_map() -> Any:
    map_path = Path(__file__).resolve().parent / "assets" / "map.py"
    spec = importlib.util.spec_from_file_location("jongmin_arena_v01_map", map_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load Jongmin arena map from {map_path}")
    module = importlib.util.module_from_spec(spec)
    old_irrlicht = sys.modules.get("pychrono.irrlicht")
    old_vehicle = sys.modules.get("pychrono.vehicle")
    if old_irrlicht is None:
        # The terrain functions do not need Irrlicht, but the handoff map imports
        # it at module import time for its standalone viewer main(). Some test or
        # headless contexts cannot initialize the Irrlicht DLL, so provide a tiny
        # placeholder while loading the terrain factory code.
        sys.modules["pychrono.irrlicht"] = types.ModuleType("pychrono.irrlicht")
    if old_vehicle is None:
        # The adapter currently skips map.py's final SCM particle zone for this
        # viewer path. Avoid importing pychrono.vehicle just to load rigid/mesh
        # terrain helpers.
        sys.modules["pychrono.vehicle"] = types.ModuleType("pychrono.vehicle")
    try:
        spec.loader.exec_module(module)
    finally:
        if old_irrlicht is None:
            sys.modules.pop("pychrono.irrlicht", None)
        else:
            sys.modules["pychrono.irrlicht"] = old_irrlicht
        if old_vehicle is None:
            sys.modules.pop("pychrono.vehicle", None)
        else:
            sys.modules["pychrono.vehicle"] = old_vehicle
    return module


def build_terrain(system: Any, terrain: Any | None = None, material: Any | None = None) -> dict[str, Any]:
    """Build Jongmin's multi-zone arena into an existing PyChrono system.

    The handed-off map uses raw PyChrono calls and creates these zones:
    flat start, rock zone, uneven mesh, obstacle gates, slope terrain, and SCM
    particle terrain. Return a small artifact dictionary so callers can inspect
    the generated SCM terrain/noise seed without knowing map.py internals.
    """
    arena = _load_arena_map()

    # Jongmin's original standalone map.py builds a ChSystemSMC and therefore
    # creates ChContactMaterialSMC for all rigid terrain bodies. The current
    # rover builder is verified with ChSystemNSC/ChContactMaterialNSC. Mixing
    # NSC rovers with SMC terrain creates collision shapes but no useful
    # collision response, so the rover sinks through the arena. In this adapter
    # process, remap terrain material construction to NSC while leaving the
    # handed-off map.py source intact.
    if hasattr(arena.chrono, "ChContactMaterialNSC"):
        arena.chrono.ChContactMaterialSMC = arena.chrono.ChContactMaterialNSC

    arena.create_segmented_floor(system)
    arena.create_outer_walls(system)
    arena.create_rough_rock_ground(system)
    arena.create_rock_zone(system)
    uneven_noise_seed = arena.create_uneven_terrain(system)
    arena.create_large_meteor_rocks(system, uneven_noise_seed)
    arena.create_obstacle_zone(system)
    arena.create_slope_terrain(system)
    # map.py's final SCM zone needs pychrono.vehicle and SMC terrain plumbing.
    # This viewer adapter is for stable visual inspection of the shared arena
    # geometry with the current NSC rover builder, so skip the SCM patch here.
    scm_terrain = None
    return {
        "factory": "jongmin_arena_v01",
        "terrain_id": getattr(terrain, "terrain_id", "jongmin_arena_v01"),
        "uneven_noise_seed": uneven_noise_seed,
        "scm_terrain": scm_terrain,
        "scm_zone_skipped": True,
    }