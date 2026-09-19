# Isaac Sim 与 NeuPAN 导航说明

> 版本状态：Isaac 场景、传感器、TF、NeuPAN 和速度控制真实运行；语义 provider、机械臂
> 动作和高层路线生成在公开默认链路中为 mock。

## 数据链

```text
living-room scene
  -> RTX LiDAR /scan_raw
  -> Isaac /odom and odom -> base_link
  -> static map -> odom bridge
  -> fixed /clicked_point -> NeuPAN /neupan_cmd_vel_raw
  -> traversal velocity gate -> /cmd_vel -> Scout Mini in Isaac
```

`map -> odom -> base_link` 必须在发布路线前可用。公开路线驱动会等待 TF 和 `/odom`，
避免 Nav2/NeuPAN 在 `odom` 尚不存在时启动任务。默认 launch 还保留
`base_link -> lidar_link` 等传感器静态变换。

## 关键文件

| 路径 | 作用 | 本机适配 |
| --- | --- | --- |
| `isaac_sim/scout_avoidance.py` | 加载场景、机器人、传感器和 ROS bridge | Isaac API 版本、GPU |
| `isaac_sim/scout_mini_isaac.urdf` | Scout Mini 仿真描述 | 替换机器人时修改 |
| `isaac_sim/ScoutMiniLidar.json` | RTX LiDAR 配置 | 频率、量程、分辨率 |
| `isaacsim-assets/InteriorAgent/kujiale_0021` | living-room 资产 | 许可与资产根目录 |
| `src/neupan_ros2/src/neupan_ros2/launch/isaac_sim.launch.py` | Isaac/NeuPAN 组合 launch | Isaac Python、显示模式 |
| `src/neupan_ros2/src/neupan_ros2/config/robots/scout` | Scout 规划参数 | footprint、checkpoint |
| `groundingdinoVLM/src/obstacle_traversal/launch/public_isaac_traversal.launch.py` | 公开固定路线入口 | 起点、目标和场景配套参数 |

## 运行

```bash
cd "$SCOUT_WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ISAACSIM_PYTHON_EXE=/path/to/isaacsim/python.sh
./run_traversal_system.sh headless:=false use_rviz:=true
```

脚本自动计算仓库路径。`ISAACSIM_PYTHON_EXE` 或 `ISAACSIM_PATH` 必须由本机设置；仓库
不提供 Isaac Sim，也不应保存开发机安装路径。

## 故障检查

```bash
ros2 topic hz /odom
ros2 topic hz /scan_raw
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /semantic_navigation/status
```

- `Invalid frame ID "odom"`：Isaac 尚未发布里程计、bridge 未加载，或存在错误的重复 TF。
- 小车不动：检查 `/clicked_point`、`/neupan_cmd_vel_raw`、`/cmd_vel` 的发布频率和速度闸状态。
- 场景为空：检查 USD 相对路径和第三方资产是否完整。
- 启动后立即失败：检查 Isaac Python 是否可执行、ROS 2 bridge 与 Isaac 版本是否匹配。

不要复制其他主机的 `build/`、`install/`、`log/`；其中嵌入绝对路径。迁移后按根 README
重新构建。运行日志、图像和 rosbag 必须按 `SECURITY.md` 处理。
