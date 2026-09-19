# Module 3: Navigation Feasibility Verification

> Public status: entity, topology, geometry, and Nav2 verification code remains; the default
> fixed-route demo does not start this module.

Module 3 performs fail-closed verification of a resolved goal and never directly controls the robot:

1. Entity and map versions match.
2. Semantic topology is reachable.
3. Goal and constrained poses are valid in the current costmap.
4. The isolated Nav2 planner can produce a globally feasible result.

Primary inputs and outputs are `/semantic_navigation/resolution`,
`/semantic_navigation/verification`, and `/semantic_navigation/verification_readiness`. The formal
chain may pass a result to Module 4 only when readiness is true and every enabled check passes.

## Formal Replacement and Host Adaptation

Adapt the map topic, global frame, robot base frame, footprint, Nav2 parameters, and keepout mask.
`map -> odom -> base_link` must exist; a static fake TF must not hide missing odometry. Research
switches isolate faults only: `SKIPPED` and `BYPASSED` are never a safe acceptance result.

Dynamic LiDAR and camera safety belong to the execution layer. A successful static plan does not
prove that the path is safe during execution.

Verification logs and costmap snapshots may expose indoor structure. Store them outside the repository
in a controlled directory and sanitize them according to the root `SECURITY.md`.
