# Scout Mini Curtain Traversal Public Demo
## (The full source code is coming soon!)


This repository is the runnable public edition of the research system. The default demo uses a
real Isaac Sim living-room scene with Scout Mini, RTX LiDAR, odometry, TF, NeuPAN, and the base
velocity chain. Private vision models, cloud reasoning, remote execution, and real arm motion are
represented by deterministic local mocks that preserve the ROS 2 contracts.

The single public acceptance target is physical simulated motion: Scout Mini starts in the
living-room, follows the fixed route, crosses the curtain, and reaches the goal region. The result
comes from Isaac Sim motion and odometry, not replayed or fabricated poses. The public demo does
not claim perception accuracy, mechanical pushing capability, or cloud-model performance.

## Public Boundary

| Status | Meaning |
| --- | --- |
| Real | Simulation, sensors, TF, planning, or control code executed by the public demo |
| Public mock | Original ROS interface is kept while the result is deterministic or a no-op |
| External adapter | Formal deployment code supplied by the operator locally or from a private package |

Default data flow:

```text
Isaac living-room -> /scan_raw, /odom, TF
                         |
                         v
fixed_route_driver -> /clicked_point -> NeuPAN -> velocity_gate -> /cmd_vel
                         |
                         +-> /semantic_navigation/status

local mock providers -> original action/topic contracts, static arm state, curtain decision
```

## Module Responsibilities

| Module or directory | Formal responsibility | Public demo |
| --- | --- | --- |
| `isaac_sim`, `isaacsim-assets` | Living-room scene, Scout physics, RTX LiDAR, camera, odometry, and TF | **Real**; Isaac Sim is installed locally and asset terms are listed in `THIRD_PARTY_NOTICES.md` |
| `src/NeuPAN`, `src/neupan_ros2` | Produce local plans and velocity from scans and goals | **Real**; only the verified public Scout checkpoint is used |
| `obstacle_traversal` | Obstacle triggers, approach, multimodal fusion, scan filtering, authorization, and velocity gating | Interfaces and local control remain; default launch uses mocks and a fixed route |
| `scout_public_mock` | Public compatibility layer | **Mock**; fixed curtain detection/decision, successful action results, static arm state, and route state machine |
| `groundingdino_vlm` | Target detection, region matching, and visual verification | **External adapter**; default launch uses `/groundingdino_vlm/verify_target` mock |
| `piper_probe` | YOLO obstacle classification, controlled probing, and arm reset | **External adapter**; default launch publishes a fixed curtain result and keeps arm actions no-op |
| `llmdecision` | Assess pushability from scene and robot state | **External adapter**; default launch uses `/llmdecision/assess_pushability` mock |
| `llmModule/Module1` | Convert natural language into a structured navigation intent | Contract and reference code remain; not started by default |
| `llmModule/Module2` | Resolve semantic entities, relations, and goal poses | Static data and query code remain; not started by default |
| `llmModule/Module3` | Verify entity, topology, geometry, and global reachability | Verification code remains; not started by default |
| `llmModule/Module4` | Convert verified results to waypoints and supervise execution | Fixed route state machine replaces the formal provider |
| `llmModule/Interfaces` | Shared cross-module messages and action contracts | **Real** |
| `scout_ros*`, `src/agx_arm_sim`, `src/ugv_sdk`, `src/livox_ros_driver2` | Robot descriptions, drivers, and third-party hardware support | Upstream dependencies; the Isaac demo does not require physical hardware |

The public mock keeps these key interfaces:

- `/groundingdino_vlm/verify_target`
- `/llmdecision/assess_pushability`
- `/piper/probe_pushability`
- `/obstacle_traversal/approach_obstacle`
- `/piper/yolo_ready`, `/piper/yolo_obstacle_result`
- `/isaac_joint_states`, `/isaac_joint_command`
- `/clicked_point`, `/semantic_navigation/status`, and traversal status topics

## Requirements

- Ubuntu 22.04 and ROS 2 Humble
- NVIDIA driver and a compatible Isaac Sim installation supplied by the user
- Python and ROS dependencies installed according to the upstream package licenses
- Living-room assets under `isaacsim-assets/InteriorAgent/kujiale_0021`

The repository may live at any path. Scripts calculate the workspace from their own location and
must not contain a developer-specific absolute path.

## Build

Do not run an unfiltered `colcon build` at the repository root: the tree contains upstream ROS 1
and ROS 2 packages with overlapping names. The public demo uses explicit paths:

```bash
cd "$SCOUT_WORKSPACE"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths \
  llmModule/Interfaces groundingdinoVLM/src src/neupan_ros2/src \
  src/scout_ros2/scout_msgs
source install/setup.bash
```

`build/`, `install/`, and `log/` are local build products and must not be committed. Rebuild after
migrating to another host. Launch scripts write ROS logs to the ignored, permission-restricted
`runtime/ros_logs/` directory by default.

## Run the Public Demo

Set local installation paths in the current shell only; never commit host-specific paths:

```bash
cd /path/to/scount-mini-ws-3
export SCOUT_WORKSPACE="$PWD"
export ISAACSIM_PYTHON_EXE=/path/to/isaac-sim/python.sh
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
./run_traversal_system.sh
```

The script defaults to `SCOUT_PUBLIC_BACKEND=mock`, does not read API keys or SSH passwords, waits
for `/odom` and the `map -> odom -> base_link` TF chain, then publishes the fixed goal. Success is
reported as `SUCCEEDED` on `/semantic_navigation/status`. The default goal is approximately
`(4.20, -0.004)` in the `map` frame; recalibrate it when the scene changes.

## Host-Specific Adaptation

| Item | Paths or source to inspect | Local adaptation | Commit? |
| --- | --- | --- | --- |
| Workspace path | Root launch scripts | Scripts calculate it; set `SCOUT_WORKSPACE` only in the shell | No absolute host paths |
| Isaac Sim | Local Isaac installation, `isaac_sim/`, `run_isaac_sim.sh` | Set `ISAACSIM_PYTHON_EXE` or `ISAACSIM_PATH`; verify Isaac and ROS bridge versions | No |
| CUDA/GPU | NVIDIA driver and Isaac runtime | Install versions matching the local GPU and Isaac release | No |
| NeuPAN checkpoint | `src/neupan_ros2/src/neupan_ros2/config/robots/scout/robot.yaml` | Replace only with a verified checkpoint and record its public source and checksum | Only verified runtime-required files |
| Living-room scene | `isaacsim-assets/InteriorAgent/kujiale_0021`, `THIRD_PARTY_NOTICES.md` | Keep relative paths or change the Isaac asset root; verify redistribution terms first | Only permitted assets |
| Physical LiDAR/CAN | `src/livox_ros_driver2/config/`, Scout driver launch/config | Set device, IP, serial, and CAN interface in ignored local config | No |
| Perception adapter | `groundingdinoVLM/src/groundingdino_vlm/config/` | External adapter supplies endpoint, model path, and weights; mock reads none | No |
| Decision adapter | `llmdecision/config/` | Inject `SCOUT_DECISION_ADAPTER=module:factory`; adapter owns endpoint, model, and credentials | No |
| Semantic adapter | `llmModule/Module1/config/`, `llmModule/Module4/config/navigation.yaml` | Configure external endpoint, model, and timeout; default demo does not start it | Sanitized config only |
| Formal route/map | Module2 maps, Module3 costmap, Module4 route parameters | Recalibrate frames, thresholds, and waypoints after scene changes | Sanitized and licensed data only |

## Replacing Mocks Gradually

Formal implementations may be added inside the corresponding module, but must preserve message
types, action/topic names, QoS, timeouts, cancellation, and failure semantics. Replace one boundary
at a time:

1. Keep `fixed_route_driver` and replace target verification with a local perception adapter.
2. Replace the pushability provider and run offline contract tests with sanitized inputs.
3. Replace the Piper probe while retaining no-op and emergency-stop paths until the physical arm is validated.
4. Enable Modules 1-4 and replace the fixed route with semantic goals and dynamic waypoints.
5. Validate each boundary in a separate launch before changing the default entry point.

`SCOUT_PUBLIC_BACKEND=real` is only an external integration entry point. The public repository does
not ship private providers; missing implementations or environment variables fail immediately and
never trigger an interactive credential prompt.

## Security and Data

Do not commit `.env` files, tokens, keys, SSH configuration, hardware addresses, rosbags, camera
frames, model responses, runtime logs, build directories, or unverified weights. Store runtime data
outside the repository or in the ignored `runtime/` directory with restricted permissions. See
[`SECURITY.md`](SECURITY.md) and [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

Run `scripts/security_scan.sh` before publishing. README files in third-party directories describe
their upstream packages; this root README defines the public mock boundary and local adaptations.

## Tests

Offline contract tests require no network or credentials:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  groundingdinoVLM/src/scout_public_mock/test \
  groundingdinoVLM/src/obstacle_traversal/test/test_four_terminal_contract.py \
  llmdecision/tests
```

Real Isaac acceptance requires a local GPU and Isaac Sim. The pass condition is actual odometry
crossing the curtain in Isaac Sim and `/semantic_navigation/status` reaching `SUCCEEDED`.
