# MoveIt2 Configurations

This directory contains independent MoveIt2 configuration packages for AgileX arms. Each package
includes kinematics, collision, and trajectory-planning configuration and can be combined with
Gazebo or Isaac Sim.

## Packages

- `nero_gripper_moveit_config`
- `piper_gripper_moveit_config`
- `piper_h_gripper_moveit_config`
- `piper_l_gripper_moveit_config`
- `piper_x_gripper_moveit_config`

## RViz2 Demo

For example, start the Nero demo and drag the interactive end-effector marker in RViz2:

```bash
ros2 launch nero_gripper_moveit_config demo.launch.py
```

## Gazebo Co-simulation

```bash
ros2 launch nero_gripper_moveit_config gazebo_moveit.launch.py
```

Gazebo and RViz2 load the model and execute the planned trajectory together.

## Isaac Sim Co-simulation

1. Start Isaac Sim and import the Nero USD asset from `agx_arm_description`.
2. Configure the ROS 2 bridge to publish `/isaac_joint_states` and subscribe to
   `/isaac_joint_command`.
3. Select `topic_based_ros2_control/TopicBasedSystem` in the MoveIt2 ros2_control configuration:

```xml
<hardware>
  <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
  <param name="joint_commands_topic">/isaac_joint_command</param>
  <param name="joint_states_topic">/isaac_joint_states</param>
</hardware>
```

4. Rebuild and source the workspace, then start MoveIt2 and RViz2:

```bash
colcon build
source install/setup.bash
ros2 launch nero_gripper_moveit_config move_group.launch.py
ros2 launch nero_gripper_moveit_config moveit_rviz.launch.py
```

The public Scout demo keeps the arm interface but uses a static mock arm; this package is the
formal planning integration point.

## License

MIT License
