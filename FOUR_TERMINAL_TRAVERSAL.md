# Four-Terminal Debugging Guide

> Status: public mock. The default demonstration only requires the root
> `run_traversal_system.sh`; this document is for observing the ROS 2 data path, not for deploying
> real providers.

All terminals must use the same `ROS_DOMAIN_ID` and RMW. The root script defaults to
`ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and `ROS_LOCALHOST_ONLY=1`. Do not
enable LAN discovery unless DDS security and network isolation are configured.

## Terminal 1: Public Demonstration

```bash
cd "$SCOUT_WORKSPACE"
export ISAACSIM_PYTHON_EXE=/path/to/isaacsim/python.sh
./run_traversal_system.sh
```

This entry point starts the real Isaac living-room scene, `/odom`, TF, LiDAR, NeuPAN, the velocity
gate, the local mock provider, and the fixed-route state machine. It does not access credentials,
start cloud models, or execute arm motions.

## Terminal 2: State Checks

```bash
cd "$SCOUT_WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1

ros2 topic echo /semantic_navigation/status
```

The normal sequence is waiting for odometry/TF, publishing the route, executing, and reaching
`SUCCEEDED`. If `WAITING_FOR_ODOM` persists, check that Isaac publishes `/odom`; for TF errors,
confirm that the `map -> odom -> base_link` chain exists and has no duplicate publisher.

## Terminal 3: Data Path

```bash
ros2 topic hz /scan_raw
ros2 topic hz /odom
ros2 topic hz /neupan_cmd_vel_raw
ros2 topic hz /cmd_vel
ros2 run tf2_ros tf2_echo map base_link
```

`/clicked_point` is published automatically by the public route driver; no manual navigation command
is required. Check mock-provider interfaces with `ros2 action list` and `ros2 topic list`.

## Terminal 4: Optional Visualization

Use `use_rviz:=true` at startup; there is no need to start a second navigation stack. Do not run
real and mock launches simultaneously, which can create duplicate action servers, duplicate TF, or
multiple velocity publishers.

## Real-Version Notes

Module1-4, real perception/decision providers, and real Piper probing are not part of the public
default chain. See the root README for replacement guidance. Credentials may only be injected by an
external secret manager; no terminal, document, or script should display, echo, or interactively
request them.
