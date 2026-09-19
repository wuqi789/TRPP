# NeuPAN ROS 2 Workspace

This compatibility file replaces the former Chinese guide with an English entry point. The
canonical workspace documentation is [`README.md`](README.md). It covers the NeuPAN planner, the
DDR minimal simulator, ROS 2 Humble setup, simulation launch files, physical Limo/Scout adaptation,
configuration, testing, and troubleshooting.

## Quick Start

```bash
source /opt/ros/humble/setup.bash
./setup.sh
./build.sh
source install/setup.bash
ros2 launch neupan_ros2 sim_complete.launch.py sim_env_config:=scenario_corridor.yaml
```

For Scout Mini hardware, keep CAN, sensor, frame, and device settings in local ignored
configuration. The repository's public Isaac demo uses the root `run_traversal_system.sh` and does
not require physical hardware.

See [`src/neupan_ros2/README.md`](src/neupan_ros2/README.md) for node parameters and topic contracts.
