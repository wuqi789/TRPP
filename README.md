# Scout Mini 门帘穿越公开演示

本仓库是论文系统的可运行公开版本。默认演示在真实 Isaac Sim living-room 场景中运行
Scout Mini、RTX LiDAR、里程计、TF、NeuPAN 和底盘速度链；未公开的视觉模型、云端推理、
远程执行和机械臂动作由确定性本地 mock 保持 ROS 2 接口兼容。

公开版本的验收目标只有一个：小车从 living-room 起点出发，沿固定路线穿过门帘并到达
目标区域。该结果来自 Isaac Sim 中的实际运动和里程计，不是回放或伪造位姿。公开版本
不用于证明真实感知精度、机械推力或云端模型效果。

## 版本边界

| 状态 | 含义 |
| --- | --- |
| 真实保留 | 公开版本直接执行的仿真、传感器、TF、规划或控制代码 |
| 公开 mock | 保留原 ROS 接口，但返回确定性结果或执行 no-op |
| 外部适配 | 正式部署时由使用者在本机或私有仓库提供，当前仓库不含实现或凭据 |

默认数据流：

```text
Isaac living-room -> /scan_raw, /odom, TF
                         |
                         v
fixed_route_driver -> /clicked_point -> NeuPAN -> velocity_gate -> /cmd_vel
                         |
                         +-> /semantic_navigation/status

local mock providers -> 原 action/topic 契约、静态机械臂状态、门帘判定
```

## 模块说明

| 模块/目录 | 应实现的正式功能 | 当前公开版本 |
| --- | --- | --- |
| `isaac_sim`、`isaacsim-assets` | living-room 场景、Scout 物理、RTX LiDAR、相机、里程计和 TF | **真实保留**；Isaac Sim 程序需本机安装，场景资产许可见第三方说明 |
| `src/NeuPAN`、`src/neupan_ros2` | 根据激光与目标点产生局部规划和速度 | **真实保留**；仅保留经上游校验的 Scout checkpoint，来源见第三方说明 |
| `obstacle_traversal` | 障碍触发、靠近、三模态融合、扫描过滤、授权和速度闸 | 接口与本地控制保留；公开启动使用 mock provider 和固定路线 |
| `scout_public_mock` | 公开版兼容层 | **公开 mock**；固定门帘检测/判断、动作成功结果、静态机械臂状态和路线状态机 |
| `groundingdino_vlm` | 目标检测、区域匹配与视觉复核 | **外部适配**；公开启动由 `/groundingdino_vlm/verify_target` mock 替代 |
| `piper_probe` | YOLO 障碍分类、机械探测、机械臂复位 | **外部适配**；公开启动发布固定门帘分类，动作是 no-op，机械臂接口仍存在 |
| `llmdecision` | 根据场景和机器人状态评估可推性 | **外部适配**；公开启动由 `/llmdecision/assess_pushability` mock 替代 |
| `llmModule/Module1` | 将自然语言解析为结构化导航意图 | 契约和参考实现保留；不在公开默认启动链中 |
| `llmModule/Module2` | 语义图实体、关系和目标位姿解析 | 数据结构和静态图实现保留；不在公开默认启动链中 |
| `llmModule/Module3` | 对实体、拓扑、几何和全局可达性做验证 | 验证代码保留；不在公开默认启动链中 |
| `llmModule/Module4` | 将验证结果转换为 waypoint 队列并监督执行 | 正式 provider 需外部适配；公开版由固定路线状态机替代 |
| `llmModule/Interfaces` | 跨模块消息和 action 契约 | **真实保留** |
| `scout_ros*`、`src/agx_arm_sim`、`src/ugv_sdk`、`src/livox_ros_driver2` | 机器人描述、驱动和第三方硬件支持 | 上游依赖；默认 Isaac 演示不要求连接实体硬件 |

公开 mock 保留的关键接口包括：

- `/groundingdino_vlm/verify_target`
- `/llmdecision/assess_pushability`
- `/piper/probe_pushability`
- `/obstacle_traversal/approach_obstacle`
- `/piper/yolo_ready`、`/piper/yolo_obstacle_result`
- `/isaac_joint_states`、`/isaac_joint_command`
- `/clicked_point`、`/semantic_navigation/status` 及 traversal 状态话题

## 环境要求

- Ubuntu 22.04 和 ROS 2 Humble
- NVIDIA 驱动及用户自行安装的兼容 Isaac Sim
- Python/ROS 依赖，按各上游包许可自行安装
- living-room 资产位于 `isaacsim-assets/InteriorAgent/kujiale_0021`

项目根目录可位于任意位置。脚本根据自身位置计算工作区，不应写死用户名目录。

## 构建

不要在根目录执行无选择的 `colcon build`，仓库包含同名 ROS 1/ROS 2 上游包。公开演示
使用以下显式路径：

```bash
cd "$SCOUT_WORKSPACE"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths \
  llmModule/Interfaces groundingdinoVLM/src src/neupan_ros2/src \
  src/scout_ros2/scout_msgs
source install/setup.bash
```

`build/`、`install/` 和 `log/` 是本机生成目录，不应提交。迁移主机后必须重新构建。启动
脚本默认把 ROS 日志写入权限受限且被忽略的 `runtime/ros_logs/`。

## 运行公开演示

先在当前 shell 中把变量指向本机安装，不要把本机绝对路径写入仓库：

```bash
cd "$SCOUT_WORKSPACE"
source /opt/ros/humble/setup.bash
export ISAACSIM_PYTHON_EXE=/path/to/isaacsim/python.sh
./run_traversal_system.sh headless:=true use_rviz:=false
```

脚本默认设置 `SCOUT_PUBLIC_BACKEND=mock`，不读取任何 API key 或 SSH 密码。路线驱动会
等待 `/odom` 与 `map -> odom -> base_link` TF 就绪，再发布固定目标。成功时：

```bash
ros2 topic echo --once /semantic_navigation/status
```

应看到终态 `SUCCEEDED`。默认目标约为 map 坐标 `(4.20, -0.004)`；这些参数与随仓库
提供的 living-room 导航层配套，替换场景后必须重新标定。

## 本机适配项

| 项目 | 需要检查的路径/来源 | 适配方式 | 是否可提交 |
| --- | --- | --- | --- |
| 工作区路径 | `run_traversal_system.sh` 及其他根启动脚本 | 脚本自动计算；手工命令只在当前 shell 设置 `SCOUT_WORKSPACE` | 否，不提交绝对路径 |
| Isaac Sim | 本机 Isaac 安装、`isaac_sim/`、`run_isaac_sim.sh` | 设置 `ISAACSIM_PYTHON_EXE` 或 `ISAACSIM_PATH`，并核对 Isaac/ROS bridge 版本 | 否 |
| CUDA/GPU | NVIDIA 驱动与本机 Isaac 运行环境 | 按本机 GPU、驱动和 Isaac 版本安装 | 否 |
| NeuPAN checkpoint | `src/neupan_ros2/src/neupan_ros2/config/robots/scout/robot.yaml` | 默认文件已按上游哈希核验；替换时同步配置并记录来源/校验值 | 仅允许已核验且运行必需的文件 |
| living-room 场景 | `isaacsim-assets/InteriorAgent/kujiale_0021`、`THIRD_PARTY_NOTICES.md` | 保持相对路径或修改 Isaac 场景根目录，并先确认再分发许可 | 仅提交许可允许的资产 |
| 实体 LiDAR/CAN | `src/livox_ros_driver2/config/`、Scout 驱动 launch/config | 在未提交的本机配置中设置设备、IP、序列号和 CAN 接口 | 否 |
| 感知 provider | `groundingdinoVLM/src/groundingdino_vlm/config/` | 外部 adapter 提供 endpoint、模型路径和权重；默认 mock 不读取 | 否 |
| 决策 provider | `llmdecision/config/` | 通过 `SCOUT_DECISION_ADAPTER=module:factory` 注入外部适配器；适配器自行管理 endpoint、模型和凭据 | 否 |
| 语义 provider | `llmModule/Module1/config/`、`llmModule/Module4/config/navigation.yaml` | 配置外部 endpoint、模型和超时；默认演示不启动 | 去敏配置可提交，凭据不可提交 |
| 正式路线/地图 | `llmModule/Module2/maps/`、Module3 costmap、Module4 route 参数 | 场景变化后重新标定坐标、占据阈值、TF frame 和 waypoint | 仅提交去敏且许可明确的数据 |

## 从 mock 逐步替换为正式实现

可以直接在对应模块内实现，但必须保持消息类型、action/topic 名称、QoS、超时和失败语义。
建议一次只替换一个边界：

1. 先保留 `fixed_route_driver`，把目标验证 action 替换为本地真实感知 adapter。
2. 替换可推性 provider，并用录制的去敏输入做离线契约测试。
3. 替换 Piper 探测实现；在实体机械臂接入前保留 no-op 与急停路径。
4. 启用 Module1-4，将固定路线替换为语义目标和动态 waypoint。
5. 每一步都先在独立 launch 中验收，再改变默认入口；不要在 mock 节点中堆叠正式逻辑。

`SCOUT_PUBLIC_BACKEND=real` 只是外部集成入口。公开仓库不保证私有 provider 开箱可用；
缺少外部实现或所需环境变量时会立即失败，也不会交互式索取凭据。

## 安全与数据

不要提交 `.env`、密钥、SSH 配置、硬件地址、rosbag、相机帧、模型响应、运行日志、
构建目录或未经来源核验的模型权重。运行数据应写到仓库外的权限受控目录，或写到已忽略的 `runtime/`。
完整规则见 [SECURITY.md](SECURITY.md)。第三方来源见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

发布前执行 `scripts/security_scan.sh`。第三方目录中的 README 保持上游说明原貌，可能包含
上游独立示例；它们不覆盖本 README 对当前公开版本、mock 边界和本机适配项的定义。

## 测试

无需网络和凭据的契约测试：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  groundingdinoVLM/src/obstacle_traversal/test/test_four_terminal_contract.py \
  groundingdinoVLM/src/scout_public_mock/test/test_public_contract.py
```

真实 Isaac 验收需要本机 GPU/Isaac Sim，成功标准是 Isaac 中的实际里程计越过门帘并且
`/semantic_navigation/status` 为 `SUCCEEDED`。
