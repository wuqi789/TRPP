# 多终端调试说明

> 版本状态：公开 mock。默认演示只需运行根目录的 `run_traversal_system.sh`；本文用于观察
> ROS 2 数据链，不是正式 provider 的部署手册。

所有终端必须使用同一 `ROS_DOMAIN_ID` 和 RMW。根脚本默认设置
`ROS_DOMAIN_ID=42`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_LOCALHOST_ONLY=1`。
除非已配置 DDS 安全和网络隔离，不要开放局域网发现。

## 终端 1：公开演示

```bash
cd "$SCOUT_WORKSPACE"
export ISAACSIM_PYTHON_EXE=/path/to/isaacsim/python.sh
./run_traversal_system.sh
```

该入口启动真实 Isaac living-room、`/odom`、TF、LiDAR、NeuPAN、速度闸，以及本地 mock
provider 和固定路线状态机。它不访问凭据、不启动云端模型，也不执行机械臂动作。

## 终端 2：状态检查

```bash
cd "$SCOUT_WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1

ros2 topic echo /semantic_navigation/status
```

正常顺序包括等待里程计/TF、发布路线、执行和 `SUCCEEDED`。若持续显示
`WAITING_FOR_ODOM`，检查 Isaac 是否发布 `/odom`；若报告 TF 错误，确认
`map -> odom -> base_link` 链存在且没有第二个同名发布者。

## 终端 3：数据通路

```bash
ros2 topic hz /scan_raw
ros2 topic hz /odom
ros2 topic hz /neupan_cmd_vel_raw
ros2 topic hz /cmd_vel
ros2 run tf2_ros tf2_echo map base_link
```

`/clicked_point` 由公开路线驱动自动发布，不需要人工发送导航指令。mock provider 的接口
可用性可通过 `ros2 action list` 和 `ros2 topic list` 检查。

## 终端 4：可选可视化

启动时使用 `use_rviz:=true` 即可，无需再启动第二套导航节点。不要同时运行正式和 mock
launch，否则可能产生重复 action server、重复 TF 或多个速度发布者。

## 正式版本说明

Module1-4、正式感知/决策 provider 和真实 Piper 探测不属于公开默认
链路。替换方式见根 README。凭据只能由外部 secret manager 注入；任何终端、文档或
脚本都不应显示、回显或交互式索取凭据。
