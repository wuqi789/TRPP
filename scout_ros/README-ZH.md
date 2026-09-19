# Scout ROS Package Guide

This file is kept for compatibility with links to the former Chinese guide. The content is now in
English; the canonical package documentation is `scout_ros/README.md`.

## Package Overview

- `scout_bringup`: launch and configuration files for Scout nodes
- `scout_base`: ROS wrapper around the Scout SDK for monitoring and control
- `scout_msgs`: Scout message definitions
- `scout_ros`: meta-package for the Scout ROS packages

The package connects the mobile base through CAN or serial transport to the SDK running on the
compute platform. The original upstream package also contains a Webots simulation integration.

## CAN-to-USB Setup

The following commands require the corresponding hardware and administrator access:

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
ip link show can0
sudo apt install can-utils
candump can0
cansend can0 001#1122334455667788
```

Use the package scripts for first-time setup and adapter re-initialization. Keep hardware-specific
interfaces, serial numbers, and addresses in local ignored configuration.

## ROS 1 Example

The upstream package targets ROS 1. For ROS 2 and the public Isaac demo, follow the root README and
the `scout_ros2` documentation instead.

```bash
roslaunch scout_bringup scout_mini_minimal.launch
roslaunch scout_bringup scout_miniomni_robot_base.launch
roslaunch scout_bringup scout_teleop_keyboard.launch
```

Reduce keyboard velocity limits before enabling a physical robot and keep a remote emergency stop
available. Never commit CAN configuration, device paths, or runtime logs.
