# Isaac Sim and NeuPAN Navigation Guide

> Status: the Isaac scene, sensors, TF, NeuPAN, and velocity control run for real; semantic
> providers, arm motions, and high-level route generation are mock in the public default chain.

## Data Path

```text
living-room scene
  -> RTX LiDAR /scan_raw
  -> Isaac /odom and odom -> base_link
  -> static map -> odom bridge
  -> fixed /clicked_point -> NeuPAN /neupan_cmd_vel_raw
  -> traversal velocity gate -> /cmd_vel -> Scout Mini in Isaac
```

`map -> odom -> base_link` must be available before the route is published. The public route driver
waits for TF and `/odom`, preventing Nav2/NeuPAN from starting a task before `odom` exists. The
default launch also retains static sensor transforms such as `base_link -> lidar_link`.

## Key Files

| Path | Purpose | Local adaptation |
| --- | --- | --- |
| `isaac_sim/scout_avoidance.py` | Loads the scene, robot, sensors, and ROS bridge | Isaac API version, GPU |
| `isaac_sim/scout_mini_isaac.urdf` | Scout Mini simulation description | Modify when replacing the robot |
| `isaac_sim/ScoutMiniLidar.json` | RTX LiDAR configuration | Rate, range, resolution |
| `isaacsim-assets/InteriorAgent/kujiale_0021` | Living-room assets | License and asset root |
| `src/neupan_ros2/src/neupan_ros2/launch/isaac_sim.launch.py` | Isaac/NeuPAN combined launch | Isaac Python, display mode |
| `src/neupan_ros2/src/neupan_ros2/config/robots/scout` | Scout planning parameters | Footprint, checkpoint |
| `groundingdinoVLM/src/obstacle_traversal/launch/public_isaac_traversal.launch.py` | Public fixed-route entry point | Start, goal, and scene-specific parameters |

## Run

```bash
cd "$SCOUT_WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ISAACSIM_PYTHON_EXE=/path/to/isaacsim/python.sh
./run_traversal_system.sh headless:=false use_rviz:=true
```

The script computes the repository path automatically. Set `ISAACSIM_PYTHON_EXE` or
`ISAACSIM_PATH` on the local machine; the repository does not provide Isaac Sim and must not store
developer-machine installation paths.

## Troubleshooting

```bash
ros2 topic hz /odom
ros2 topic hz /scan_raw
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /semantic_navigation/status
```

- `Invalid frame ID "odom"`: Isaac has not published odometry, the bridge is not loaded, or a
  duplicate TF publisher is misconfigured.
- Robot does not move: check the publish rates and velocity-gate state for `/clicked_point`,
  `/neupan_cmd_vel_raw`, and `/cmd_vel`.
- Empty scene: check the USD relative path and third-party asset completeness.
- Immediate startup failure: check that Isaac Python is executable and that the ROS 2 bridge and
  Isaac versions are compatible.

Do not copy `build/`, `install/`, or `log/` from another host; they embed absolute paths. Rebuild
after migration according to the root README. Handle runtime logs, images, and rosbags according to
`SECURITY.md`.
