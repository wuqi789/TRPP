# Semantic Navigation Modules

> Public status: Module 1-4 contracts, static semantic data, and reference implementations remain,
> but the default curtain demo does not start external semantic providers. `scout_public_mock` and
> `fixed_route_driver` provide deterministic waypoints and execution status.

## Formal Pipeline

```text
/user_instruction
  -> Module1: language intent
  -> Module2: semantic entities and pose resolution
  -> Module3: entity, topology, geometry, and global-planning verification
  -> Module4: waypoint proposal, validation, and queue execution
  -> /clicked_point -> NeuPAN
```

`Interfaces` defines shared ROS types. `AffordanceMapping` is an optional pushability semantic layer.
External reasoning for Modules 1 and 4 is a formal adapter, not a default public capability.

| Directory | Public content | Default demo |
| --- | --- | --- |
| `Interfaces` | Message and action contracts | Used |
| `Module1` | Parser, provider boundary, and mock tests | Not started |
| `Module2` | Static semantic graph and queries | Not started |
| `Module3` | Verification logic and isolated Nav2 configuration | Not started |
| `Module4` | Route-provider boundary and execution supervision | Replaced by fixed route |
| `AffordanceMapping` | Pushability map interface | Not started |

## Formal Integration

Formal code may be implemented directly in each module, provided that
`semantic_navigation_interfaces`, topic/action names, QoS, task de-duplication, cancellation, and
terminal-state semantics remain unchanged. Replace Modules 1 through 4 one at a time and run
contract tests with offline, sanitized data first.

Model endpoints, API credentials, private maps, and diagnostic directories are host adaptations.
Inject credentials into the process through a secret manager, never into YAML or command arguments.
Store route images, model responses, and execution audits outside the repository.

## Build and Test

Dependency preparation may access the network and create `.venv`; do not commit those files:

```bash
cd "$SCOUT_WORKSPACE/llmModule"
source /opt/ros/humble/setup.bash
./scripts/prepare_scout_dependencies.sh
source setup_scout_llm.sh
colcon build --symlink-install --packages-select \
  semantic_navigation_interfaces semantic_navigation_adapters \
  llm_semantic_affordance llm_module1 llm_module2 llm_module3 llm_module4
```

The public fixed-route demo runs `run_traversal_system.sh` from the repository root and does not
start cloud providers. See each module README for its boundary and host adaptations.
