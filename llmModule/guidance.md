# Module1-4 Real Integration Guide

> The current public default demonstration does not start external Module1-4 providers. This guide
> only explains how to integrate real implementations step by step; it contains no endpoints,
> credentials, private models, or production data.

## Integration Order

1. Fix the Isaac scene, `/map`, `/odom`, and `map -> odom -> base_link` TF contracts.
2. Enable Module2 and use a redacted static semantic map to verify entity IDs, frames, and poses.
3. Enable Module3, wait for all readiness checks, and verify entities, topology, geometry, and the
   planner.
4. Enable Module1 with an offline mock and verify the natural-language-to-canonical-intent schema.
5. Enable Module4 with a local fixed-route provider and verify preemption, cancellation, hold, FIFO,
   and terminal states.
6. Replace the Module1/Module4 providers separately at the end; change only one network boundary at
   a time.

## Required Constraints

- All nodes must use the same ROS domain, RMW, and simulation clock.
- Module4 must not publish motion goals until Module3 passes.
- Validate waypoint frames, bounds, traversability, and task association.
- A model-generated route is not proof of collision safety; NeuPAN and the execution watchdog must
  still operate.
- Fail closed on provider timeouts, invalid responses, missing TF, duplicate nodes, or uncertain
  state.
- Real packages must preserve `semantic_navigation_interfaces` message/action compatibility.

## Local Adaptation

| Area | Items to verify |
| --- | --- |
| Isaac | Local Python, ROS bridge, scene assets, and spawn pose |
| Module2 | Map revision, entity IDs, frames, and target poses |
| Module3 | Map topic, costmap, footprint, TF, and Nav2 parameters |
| Module1/4 | External adapter, model name, endpoint, CA, timeouts, and schema |
| Runtime data | Controlled directory outside the repository, permissions, retention, and redaction |

Inject credentials only through the deployment platform's secret manager. Do not assign or echo them
in command examples; fail immediately when they are missing. Diagnostic graphs, route summaries, and
execution records default to `$SCOUT_WORKSPACE/runtime/` (ignored). Public code does not persist raw
provider responses, prompts, or tracebacks; production deployments should configure an external
directory.

## Verification

Run each module's offline tests first, then observe intent, resolution, verification, status, and
`/clicked_point` stage by stage in an isolated ROS domain. Real results must distinguish offline mock,
Isaac integration, and physical-hardware evidence; mock success cannot replace real-provider or
mechanical-safety acceptance.
