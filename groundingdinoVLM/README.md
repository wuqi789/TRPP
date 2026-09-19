# Target Verification and Obstacle Traversal Interfaces

> Public status: ROS 2 messages, actions, traversal control, and safety gates remain available.
> The private perception backend and physical Piper probing are not distributed; the default path
> uses `scout_public_mock`.

## Responsibilities

- `groundingdino_vlm`: produce target identity, confidence, and verification evidence from
  synchronized images and target regions.
- `piper_probe`: classify obstacles, perform controlled probing, and reset the arm safely.
- `obstacle_traversal`: approach obstacles, fuse multimodal results, authorize local scan filtering,
  and limit speed.
- `scout_public_mock`: provide deterministic public-demo providers, static arm interfaces, and the
  fixed route driver.
- `*_interfaces`: shared message, service, and action contracts.

The public entry point does not load model weights, access the network, or read credentials. It keeps:

- `/groundingdino_vlm/verify_target`
- `/llmdecision/assess_pushability`
- `/piper/probe_pushability`
- `/obstacle_traversal/approach_obstacle`
- `/piper/yolo_ready`, `/piper/yolo_obstacle_result`
- `/isaac_joint_states`, `/isaac_joint_command`

The mock arm publishes static joint states and accepts joint commands without moving. Mock results
validate interfaces, state machines, and the navigation loop; they are not evidence of perception
accuracy or mechanical safety.

## Build and Run

Build from the repository root as described in the root `README.md`, then set the local Isaac Sim
interpreter and launch:

```bash
export ISAACSIM_PYTHON_EXE=/path/to/isaacsim/python.sh
./run_traversal_system.sh
```

The default is `SCOUT_PUBLIC_BACKEND=mock`. Real mode is only an external adapter entry point and
private implementations are not included. Inject required values from a secret manager; never put
credentials in documentation, YAML, command arguments, or shell history. Missing configuration
fails immediately without an interactive prompt.

## Replacing the Formal Providers

Implement a formal provider inside its package or as a private ROS package without bypassing the
downstream contract. Preserve action names, message types, QoS, `request_id` correlation, timeout,
cancellation, and fail-closed behavior. Keep model paths, endpoints, device identifiers, and hardware
parameters in ignored host configuration.

## Tests

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  groundingdinoVLM/src/scout_public_mock/test \
  groundingdinoVLM/src/obstacle_traversal/test/test_four_terminal_contract.py
```

Test placeholders are not credentials. Tests must not print environment variables; handle logs and
images according to the root `SECURITY.md`.
