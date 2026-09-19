# Module 4: Waypoint Generation and Execution Supervision

> Public status: execution contracts and reference code remain; the default curtain demo uses
> `fixed_route_driver` instead of a dynamic route provider and does not access cloud models.

Formal Module 4 converts a goal verified by Module 3 into ordered waypoints, validates the output,
and drives NeuPAN point by point through `/clicked_point`. NeuPAN remains responsible for local motion
and obstacle avoidance.

## Main Interfaces

| Direction | Name | Type |
| --- | --- | --- |
| Input | `/semantic_navigation/verification` | `VerificationResult` |
| Input | `/semantic_validation/map` | `nav_msgs/OccupancyGrid` |
| Input | `/semantic_mapping/pushability_map` | `PushabilityMap` |
| Action | `/semantic_navigation/execute` | `ExecuteNavigation` |
| Output | `/semantic_navigation/status` | `NavigationTaskStatus` |
| Output | `/semantic_navigation/readiness` | `SystemReadiness` |
| Output | `/semantic_navigation/execution_route` | `nav_msgs/Path` |
| Output | `/clicked_point` | `PointStamped` |

Route-provider output must pass schema, bounds, frame, free-space, connectivity, and task-association
checks. A visual polyline is not collision-safety evidence; local control and an execution watchdog
remain required.

## Public Mock and Formal Replacement

The public fixed goal matches the living-room assets and declares success using TF distance and a
stability interval. Formal replacements must preserve topic/action names, QoS, task de-duplication,
preemption, cancellation, hold-goal, timeout, and terminal-state semantics. A route model may be
provided as an independent adapter; private network calls do not belong in the fixed-route mock.

Provider endpoint, model, credentials, output directory, and map parameters are host adaptations.
Route images, point adjustments, and execution audits may reveal indoor layouts; keep them outside
the repository in a restricted directory with retention expiry.

## Tests

```bash
cd "$SCOUT_WORKSPACE/llmModule"
source setup_scout_llm.sh
colcon test --packages-select llm_module4
colcon test-result --verbose
```

Report formal-provider and Isaac results separately. A mock pass proves only the interface and state
machine loop.
