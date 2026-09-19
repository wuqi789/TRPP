# Module 2: Semantic Graph and Goal Resolution

> Public status: static graph structures, the Scout living-room example map, and query logic remain;
> the default fixed-route demo does not start this module.

Module 2 maps Module 1's structured intent to scene entities, relations, and a goal pose. It does not
create metric paths, perform local obstacle avoidance, or control the base.

## Interfaces and Data

- Services for entity, relation, pose, and topology queries
- Navigation intent input and a semantic goal with a unique entity ID and pose
- `maps/scout_kujiale_0021.yaml`: public static living-room example
- `maps/DCT_hospital_demo.yaml`: historical regression data, not the default scene

Graph nodes may represent rooms, objects, waypoints, doors, and corridors. Topological reachability
does not prove physical traversability; Module 3 must still verify the current map and TF.

## Formal Replacement and Host Adaptation

When replacing a scene, update entity IDs, frames, map origin, and relation data. Remove resident
details and device serial numbers. Keep private maps and database credentials outside the repository.
Every output pose must declare its frame and follow the `map`/`odom` convention used by Module 3 and
Isaac Sim.

## Tests

```bash
cd "$SCOUT_WORKSPACE/llmModule/Module2"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_graph.py
```
