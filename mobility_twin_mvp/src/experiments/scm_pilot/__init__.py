"""Scout-to-Main SCM transfer pilot.

Research question (see 우주로버 아이디어.pdf, 01 Executive Summary):
    Does a small rover's (scout_v01) measured slip/sinkage/torque/energy
    response on an SCM soil patch let us predict a differently-sized rover's
    (main_v01) response on the same patch better than a slope-only or
    identity baseline?

This package is intentionally decoupled from the main Contract v2 app
(no Streamlit, no risk map, no full arena) -- see
docs/SCOUT_MAIN_SCM_PILOT_PLAN.md for why, and for the current status. It
reuses rover_models/scout_v01, rover_models/main_v01,
src/chrono/rover_factory.py, and src/chrono/terrain_factory.py rather than
building a parallel rover/terrain representation.
"""
