# agx_arm_sim

AgileX ROS 2 simulation and development packages for arm algorithm development, simulation tests,
and visualization. The repository combines arm descriptions, motion-planning configurations, and
simulation adapters so that a model can be checked in RViz2, MoveIt2, Gazebo, or Isaac Sim.

## Repository Layout

```text
agx_arm_sim/
├── agx_arm_description/   # URDF/Xacro descriptions and USD assets
├── Moveit2/               # MoveIt2 planning configurations
├── realsense2_description/ # RealSense camera descriptions
└── .gitmodules            # Model-resource submodules
```

The description package provides parameterized models, end-effector variants, camera stands, and
RViz2 visualization. MoveIt2 packages provide kinematics, collision checking, and trajectory
planning. USD models can be imported into Isaac Sim.

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/agilexrobotics/agx_arm_sim.git
cd agx_arm_sim
git submodule update --init --recursive
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs \
  ros-humble-topic-based-ros2-control ros-humble-moveit*
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Supported Arm Models

| Identifier | Model | End-effector families |
| --- | --- | --- |
| `piper` | Piper | none, electric gripper, Revo2, teach pendant, Pika2 |
| `piper_h` | Piper H | none, electric gripper, Revo2, teach pendant, Pika2 |
| `piper_l` | Piper L | none, electric gripper, Revo2, teach pendant, Pika2 |
| `piper_x` | Piper X | none, electric gripper, Revo2, teach pendant, Pika2 |
| `nero` | Nero | none, electric gripper, Revo2, teach pendant, Pika2 |
| `revo2` | Revo2 hand | Configure left or right hand independently |

## Supported Simulation Targets

| Target | Support |
| --- | --- |
| Isaac Sim | USD arm models for high-fidelity physics and embodied-AI experiments |
| MoveIt2 | Motion planning, collision checking, and trajectory validation |
| RViz2 | Fast model and joint visualization without a full simulator |

| Model | Isaac Sim | MoveIt2 | MuJoCo | Gazebo |
| --- | --- | --- | --- | --- |
| `piper` | Yes | Yes | Yes | Yes |
| `piper_h` | Yes | Yes | No | Yes |
| `piper_l` | Yes | Yes | No | Yes |
| `piper_x` | Yes | Yes | No | Yes |
| `nero` | Yes | Yes | Yes | Yes |

## License

MIT License
