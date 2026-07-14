"""Rigid-terrain Scout-to-Main transfer pilot.

Research question (same as src/experiments/scm_pilot, narrowed to rigid
terrain): does scout_v01's measured slip/torque/energy response predict
main_v01's response better than a slope-only or identity baseline?

This pilot exists because src/experiments/scm_pilot is blocked on a
pychrono.vehicle DLL import failure (see docs/ENVIRONMENT_SETUP.md). Rigid
terrain only needs pychrono core, so this pilot can run today. SCM work is
paused, not abandoned -- see docs/SCOUT_MAIN_SCM_PILOT_PLAN.md.

Reuses rover_models/scout_v01, rover_models/main_v01,
src/chrono/rover_factory.py, and src/chrono/terrain_factory.py unmodified.
Slope is applied by tilting gravity and the single obstacle condition is
built with a local plain-pychrono helper (see scenario.py) specifically so
this pilot does not need to modify terrain_factory.py.
"""
