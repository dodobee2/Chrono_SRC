"""Vendored copy of handoff/rover_module_v01/code/src (Hojin, 2026-07-14).

Source: handoff/rover_module_v01/code/src/{rover_schema.py,rover_builder.py}.
Only change from the original: rover_builder.py's bare `import rover_schema`
was changed to a relative import so it works as part of the src.chrono
package. Do not edit the physics/spec logic here -- change
src/chrono/rover_factory.py instead, which adapts Contract v2 RoverSpec into
this module's RoverSpec/SimConfig.
"""
