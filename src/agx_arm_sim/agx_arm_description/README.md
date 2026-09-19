# agx_arm_description

ROS 2 description package for AgileX arms. The parameterized Xacro entry point
`urdf/agx_arm_description.urdf.xacro` supports arm models, end effectors, camera stands, and
RealSense D435 combinations. It publishes a URDF for RViz2 and includes USD models for Isaac Sim.

## Supported Models

| `arm_type` | Model | End-effector values |
| --- | --- | --- |
| `piper` | Piper | `none`, `gripper`, `revo2_left`, `revo2_right`, `teach`, `pika` |
| `piper_h` | Piper H | Same values |
| `piper_l` | Piper L | Same values |
| `piper_x` | Piper X | Same values |
| `nero` | Nero | Same values |
| `revo2` | Revo2 hand | Select `revo2_side:=left` or `right` |

## Layout

```text
agx_arm_description/
├── agx_arm_urdf/                 # agilexrobotics/agx_arm_urdf submodule
├── meshes/realsense_mid_stand.dae
├── urdf/agx_arm_description.urdf.xacro
├── urdf/teach_pendant.urdf.xacro
├── urdf/pika2_gripper.urdf
├── urdf/*.usd
├── launch/display.launch.py
├── config/arm_config.yaml
└── rviz/default.rviz
```

## Dependencies and Build

```bash
sudo apt install ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-xacro

cd ~/ros2_ws/src
git clone https://github.com/agilexrobotics/agx_arm_sim.git
cd agx_arm_description
git submodule update --init --recursive
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Launch Parameters

| Parameter | Default | Values | Description |
| --- | --- | --- | --- |
| `arm_type` | `piper` | `piper`, `piper_h`, `piper_l`, `piper_x`, `nero`, `revo2` | Arm model |
| `end_effector` | `none` | `none`, `gripper`, `revo2_left`, `revo2_right`, `teach`, `pika` | End effector; ignored for `revo2` |
| `revo2_side` | `right` | `left`, `right` | Revo2 hand side |
| `with_camera_stand` | `false` | `true`, `false` | Load the camera stand |
| `with_camera` | `false` | `true`, `false` | Load RealSense D435; requires the stand |
| `use_gui` | `true` | `true`, `false` | Start the joint slider GUI |
| `rviz_config` | Built-in | Any `.rviz` path | Custom RViz2 configuration |

## Common Commands

```bash
ros2 launch agx_arm_description display.launch.py
ros2 launch agx_arm_description display.launch.py arm_type:=piper_h
ros2 launch agx_arm_description display.launch.py arm_type:=piper end_effector:=gripper
ros2 launch agx_arm_description display.launch.py arm_type:=nero end_effector:=revo2_right
ros2 launch agx_arm_description display.launch.py arm_type:=revo2 revo2_side:=left
ros2 launch agx_arm_description display.launch.py arm_type:=piper end_effector:=pika
ros2 launch agx_arm_description display.launch.py arm_type:=piper end_effector:=teach
ros2 launch agx_arm_description display.launch.py arm_type:=nero end_effector:=gripper \
  with_camera_stand:=true with_camera:=true
```

The camera stand is mounted to `gripper_base` and selects model-specific offsets automatically.

## Xacro from Another Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory("agx_arm_description"),
        "urdf", "agx_arm_description.urdf.xacro")
    description = xacro.process_file(
        xacro_file,
        mappings={"arm_type": "piper", "end_effector": "gripper",
                  "with_camera_stand": "true", "with_camera": "true"}).toxml()
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}])
    ])
```

The Xacro file can also be converted directly after sourcing ROS 2:

```bash
ros2 run xacro xacro urdf/agx_arm_description.urdf.xacro \
  arm_type:=nero end_effector:=gripper \
  with_camera_stand:=true with_camera:=true -o nero_full.urdf
```

## USD Assets

USD arm models are stored under `urdf/` and can be imported into Isaac Sim without an additional
format conversion. See the root `THIRD_PARTY_NOTICES.md` before redistributing assets.

## License

MIT License
