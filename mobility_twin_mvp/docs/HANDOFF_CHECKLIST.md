# Integration Contract v2 Handoff Checklist

## Hojin Rover Handoff

- `rover_id`
- mass, inertia, and CG
- wheel count, radius, and width
- wheel joint axes
- driven wheel list
- command type
- torque and speed limits
- visual asset
- collision asset
- coordinate convention
- model entry point or factory URI
- build/run requirements

Required file:

```text
rover_models/<rover_id>/rover.yaml
```

## Jongmin Terrain Handoff

- `terrain_id`
- geometry source type
- mesh, heightmap, or factory URI
- coordinate convention and origin
- dimensions
- obstacle pose and dimensions
- collision asset
- material IDs
- friction/restitution or SCM parameters
- random seed
- build/run requirements

Required files:

```text
terrain_scenarios/<terrain_id>/terrain.yaml
terrain_materials/<material_id>.yaml
contact_pairs/<contact_pair_id>.yaml
```

## Scout Observation Handoff

- `observation_id`
- linked `terrain_id`
- linked `scout_rover_id`
- linked `control_profile_id`
- timestamp
- grid and pose
- slope, roughness, obstacle height, and gap width
- speed, slip, sinkage, torque, COT, vibration
- geometry and response confidence
- `source_type`

Required file:

```text
observations/<observation_id>/observation.yaml
```

## Validation Notes

All dimensions, distances, speeds, forces, torques, and times use SI units. Confidence values must be in `0..1`. Asset URIs should be relative to the repository root or the scenario directory unless intentionally marked as a placeholder URI.

