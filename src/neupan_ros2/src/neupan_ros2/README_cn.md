# NeuPAN ROS 2 Node

This compatibility file is now English. The full node documentation is in
[`README.md`](README.md), including simulation and physical-robot launch files, Scout Mini safety
gates, parameters, topics, TF requirements, visualization controls, and testing.

The node consumes scans and goals, publishes local plans and velocity references, and must stop when
the robot-state safety gate is not satisfied. Keep CAN interfaces, sensor calibration, device IDs,
model paths, and runtime logs in local ignored configuration.

For the public release, use the root fixed-route Isaac Sim launch. It retains the real TF, odometry,
LiDAR, NeuPAN, and velocity chain while replacing private providers with local mocks.
