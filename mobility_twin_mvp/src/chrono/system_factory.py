"""Single entry point for creating a Chrono system and contact materials.

This project has hit the same bug class twice in one day (2026-07-14):
  1. smoke_scenario.py forgot to call SetCollisionSystemType -- bodies fell
     through each other with no collision response at all.
  2. terrain_factory.py's rigid floor used ChContactMaterialSMC while every
     other verified-working piece of Chrono code in this project (vendored
     rover_builder.py, smoke_scenario.py) uses ChSystemNSC -- the mismatched
     contact method meant the floor had a collision shape but applied no
     real force, so rovers fell straight through it (see
     src/experiments/rigid_transfer_pilot's real trajectory output before
     the fix: z falling to -3.86m, contact_count=0 throughout).

Both bugs looked structurally identical: a collision *shape* existed but no
collision *force* was ever applied, and nothing caught it until a real
multi-second simulation was actually run and its trajectory inspected --
existence checks like `GetCollisionModel() is not None` do not catch this
class of bug.

Use make_nsc_system()/make_nsc_contact_material() instead of calling
chrono.ChSystemNSC()/ChContactMaterialNSC() directly when creating a new
system or material in this project, so the collision system and contact
method are never forgotten or mismatched again. ChSystemNSC is the only
system type verified to work in this project; if SMC is ever needed, every
body's contact material in that system must be SMC too -- do not mix them.
"""

from __future__ import annotations

from typing import Any


def make_nsc_system(gravity_mps2: tuple[float, float, float] = (0.0, 0.0, -9.81)) -> Any:
    """Creates a ChSystemNSC with Bullet collision and gravity already set."""
    import pychrono as chrono

    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(*gravity_mps2))
    return system


def make_nsc_contact_material(friction: float = 0.8, restitution: float = 0.02) -> Any:
    """Creates a ChContactMaterialNSC -- the only contact method verified to
    work with make_nsc_system() in this project.
    """
    import pychrono as chrono

    material = chrono.ChContactMaterialNSC()
    material.SetFriction(friction)
    material.SetRestitution(restitution)
    return material
