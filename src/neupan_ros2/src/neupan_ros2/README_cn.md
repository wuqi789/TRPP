# NeuPAN ROS2

<a href="https://ieeexplore.ieee.org/abstract/document/10938329"><img src='https://img.shields.io/badge/PDF-IEEE-brightgreen' alt='PDF'></a>
<a href="https://arxiv.org/pdf/2403.06828.pdf"><img src='https://img.shields.io/badge/PDF-Arxiv-brightgreen' alt='PDF'></a>
<a href="https://youtu.be/SdSLWUmZZgQ"><img src='https://img.shields.io/badge/Video-Youtube-blue' alt='youtube'></a>
<a href="https://www.bilibili.com/video/BV1Zx421y778/?vd_source=cf6ba629063343717a192a5be9fe8985"><img src='https://img.shields.io/badge/Video-Bilibili-blue' alt='youtube'></a>
<a href="https://hanruihua.github.io/neupan_project/"><img src='https://img.shields.io/badge/Website-NeuPAN-orange' alt='website'></a>
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)

**中文版** | [**English**](README.md)

---

## 🌟 项目概述

**NeuPAN ROS2** 是 [NeuPAN](https://github.com/hanruihua/neupan) 的 ROS2 封装包，NeuPAN 是一个先进的端到端模型学习框架，用于基于点云的机器人直接导航。本软件包实现了 NeuPAN 强大的导航能力与 ROS2 机器人系统的无缝集成。

### 主要特性

- ✨ **端到端学习**：基于点云的直接导航，无需显式建图
- 🚀 **实时性能**：高效的神经网络推理，实现自主导航
- 🤖 **多平台支持**：同时支持仿真环境和实体机器人（Limo、自定义平台）
- 🔧 **灵活配置**：基于 YAML 的易于定制的参数系统
- 📡 **ROS2 原生**：与 ROS2 Humble 生态系统完全集成

---

## 📦 前置条件与依赖

### 系统要求

- **操作系统**：Ubuntu 22.04 LTS
- **ROS2 发行版**：Humble Hawksbill
- **Python**：3.10 或更高版本

### 核心依赖

#### ROS2 软件包
```bash
# ROS2 Humble（推荐完整桌面版安装）
sudo apt install ros-humble-desktop-full

# 额外的 ROS2 软件包
sudo apt install ros-humble-rviz2 \
                 ros-humble-tf2-ros \
                 ros-humble-sensor-msgs \
                 ros-humble-nav-msgs \
                 ros-humble-geometry-msgs \
                 ros-humble-visualization-msgs
```

#### Python 依赖

⚠️ **重要**：NeuPAN 需要 numpy < 2.0

```bash
# PyTorch（根据您的配置选择 CPU 或 GPU 版本）
# 访问 https://pytorch.org 了解安装选项
pip3 install torch torchvision

# NeuPAN 核心库
pip3 install neupan

# 其他 Python 包（注意 numpy 版本要求）
pip3 install "numpy<2.0" scipy matplotlib pyyaml
```

详细的 Python 环境设置请参考官方 NeuPAN 仓库：
**https://github.com/hanruihua/NeuPAN**

### 可选依赖

- **仿真环境**：[ddr_minimal_sim](../ddr_minimal_sim)（包含在本工作空间中）
- **Limo 机器人**：AgileX Limo ROS2 驱动软件包

---

## 🚀 安装步骤

> **注意**：此软件包现已成为 NeuPAN ROS2 工作空间的一部分。完整安装说明请参见[工作空间 README](../../README.md)。

### 快速安装（作为工作空间的一部分）

此软件包已与 ddr_minimal_sim 一起包含在 NeuPAN ROS2 工作空间中。安装步骤：

```bash
# 克隆工作空间
git clone https://github.com/KevinLADLee/neupan_ros2.git
cd neupan_ros2

# 安装系统依赖
chmod +x setup.sh
./setup.sh

# 安装 Python 依赖（参见上述要求）
pip3 install neupan
pip3 install torch torchvision
pip3 install "numpy<2.0" scipy matplotlib pyyaml

# 构建工作空间
chmod +x build.sh
./build.sh

# Source 工作空间
source install/setup.bash
```

详细的安装、故障排除和使用说明，请参阅[工作空间 README](../../README.md)。

---

---

## 📖 快速开始

### 🎮 1. 仿真模式（使用 ddr_minimal_sim）

启动包含 NeuPAN 规划器的完整仿真环境：

```bash
# 激活工作空间
source ~/neupan_ws/install/setup.bash

# 使用默认环境启动
ros2 launch neupan_ros2 sim_complete.launch.py

# 或指定自定义环境配置
ros2 launch neupan_ros2 sim_complete.launch.py sim_env_config:=scenario_corridor.yaml use_rviz:=true
```

**可用环境配置**：
- `scenario_maze.yaml`：迷宫场景（默认）
- `scenario_corridor.yaml`：走廊场景
- `scenario_narrow_passage.yaml`：窄通道场景
- `scenario_u_trap.yaml`：U 型陷阱场景
- `scenario_polygon_random.yaml`：随机多边形障碍场景
- `scenario_empty.yaml`：空旷场景

### 🤖 2. 实体机器人模式（Limo 平台）

用于 AgileX Limo 差速驱动机器人：

```bash
# 在 Limo 机器人上启动 NeuPAN
ros2 launch neupan_ros2 limo.launch.py

# 不启动 RViz
ros2 launch neupan_ros2 limo.launch.py use_rviz:=false
```

> **注意**：本软件包已针对 [AgileX Limo ROS2](https://www.agilex.ai/education/18) 机器人进行优化。如需了解该平台信息，请联系我们的合作伙伴：sales@hive-matrix.com。

### Scout Mini 实机部署

当前工作空间中的 Scout 启动文件会同时启动 `can0` 底盘驱动、Livox
MID360S、点云转 LaserScan、SLAM Toolbox、OctoMap、NeuPAN 和 RViz2。
雷达固定外参为 `base_link -> livox_frame: (0.21, 0, 0.17)`。

在工作空间根目录一键启动：

```bash
./run_scout.sh
```

首次联调或只建图、不允许自主运动时：

```bash
./run_scout.sh enable_motion:=false
```

脚本会检查 `can0`；仅当接口未启动或不是 `500 kbit/s` 时请求 `sudo`
权限进行配置。RViz 固定坐标系为 `map`，二维栅格显示 `/map`，三维占据点云
显示 `/octomap_point_cloud_centers`。选择 RViz 的 `Publish Point` 工具即可向
`/clicked_point` 发布单个目标。

Scout 自主运动还必须同时满足：连续 5 帧 `vehicle_state=0`、
`control_mode=1`、`error_code=0`，且 `/scout_status` 不超过 `0.5 s` 未更新。
任何条件失效都会持续输出零速度。

推荐的实车操作顺序：

1. 打开遥控器并释放急停，遥控接收器必须在整个运行期间保持在线。
2. 运行 `./run_scout.sh`，先在 RC 模式下遥控车辆完成建图。尚未发布目标时，
   NeuPAN 不会发送运动指令。
3. 准备自主导航时，将底盘切换到 CAN 控制模式；等待终端显示
   `Scout safety gate ready: normal state, CAN mode, no errors`。
4. 在 RViz 中选择 `Publish Point` 并点击 `/map` 上的目标。状态门任何时候关闭，
   NeuPAN 都会立即改为持续发送零速度。

若状态为 `vehicle_state=2, control_mode=3, error_code=4`，含义是底盘异常、
RC 模式且遥控信号丢失。此时应检查遥控器供电/配对、接收器连接和急停，不能通过
关闭软件安全门绕过。可在另一终端用下面的命令复查：

```bash
source $SCOUT_WORKSPACE/install/setup.bash
ros2 topic echo /scout_status --once
```

### ⚙️ 3. 自定义配置

```bash
# 独立启动 NeuPAN 节点（不包含 ddr_minimal_sim）
ros2 launch neupan_ros2 simulation.launch.py
```

---

## 🎯 配置说明

### 配置文件

配置文件位于：
```
config/
└── robots/
    ├── simulation/
    │   ├── robot.yaml           # 仿真机器人 ROS 参数
    │   ├── planner.yaml         # 仿真规划器参数
    │   └── models/
    │       └── dune_model_5000.pth
    ├── limo/
    │   ├── robot.yaml           # Limo 机器人 ROS 参数
    │   ├── planner.yaml         # Limo 规划器参数
    │   └── models/
    │       └── dune_model_5000.pth
    ├── scout/
    │   ├── robot.yaml           # Scout 机器人 ROS 参数
    │   ├── planner.yaml         # Scout 规划器参数
    │   └── models/
    │       └── dune_model_5000.pth
    └── ranger/
        ├── robot.yaml
        ├── planner.yaml
        └── models/
            └── dune_model_5000.pth
```

### 关键参数

#### 核心参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `use_sim_time` | 使用仿真时间 | `true`/`false` |
| `planner_config_file` | 规划器配置文件 | `planner.yaml` |
| `dune_checkpoint_file` | 神经网络模型文件 | `models/dune_model_5000.pth` |
| `map_frame` | 全局坐标系 | `map` |
| `base_frame` | 机器人基座坐标系 | `base_link` |
| `scan_range_max` | 激光扫描最大距离（米） | `5.0` |
| `scan_range_min` | 激光扫描最小距离（米） | `0.01` |
| `ref_speed` | 参考导航速度（米/秒） | `0.5` |
| `collision_threshold` | 碰撞避障阈值（米） | `0.01` |

#### 话题配置参数（新增）

所有话题名称现在均可通过 ROS 参数进行配置，以实现灵活集成：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `cmd_vel_topic` | 速度命令输出话题 | `/neupan_cmd_vel` |
| `scan_topic` | 激光扫描输入话题 | `/scan` |
| `plan_input_topic` | 全局路径输入话题 | `/plan` |
| `clicked_point_topic` | Publish Point 目标点输入话题 | `/clicked_point` |
| `plan_output_topic` | 优化轨迹输出话题 | `/neupan_plan` |
| `ref_state_topic` | 参考状态输出话题 | `/neupan_ref_state` |
| `initial_path_topic` | 初始路径可视化话题 | `/neupan_initial_path` |
| `dune_markers_topic` | DUNE 可视化标记话题 | `/dune_point_markers` |
| `robot_marker_topic` | 机器人轮廓标记话题 | `/robot_marker` |
| `nrmp_markers_topic` | NRMP 可视化标记话题 | `/nrmp_point_markers` |

#### 可视化控制参数（v0.3.0 新增）

控制可视化标记以优化低功耗平台上的性能：

| 参数 | 说明 | 默认值（仿真） | 默认值（实物） |
|------|------|----------------|----------------|
| `enable_visualization` | 所有可视化标记的主开关 | `false` | `true` |
| `enable_dune_markers` | 启用 DUNE 点云可视化 | `true` | `true` |
| `enable_nrmp_markers` | 启用 NRMP 点云可视化 | `true` | `true` |
| `enable_robot_marker` | 启用机器人轮廓可视化 | `true` | `true` |

**性能影响：**
- 禁用可视化（`enable_visualization: false`）：嵌入式平台上可减少约 5-10% 的 CPU 占用
- 选择性标记：禁用 DUNE/NRMP 仅保留机器人轮廓以最小化开销

**配置示例：**
```yaml
# 嵌入式平台的最小可视化配置
enable_visualization: true
enable_dune_markers: false    # 禁用 CPU 密集型点云
enable_nrmp_markers: false
enable_robot_marker: true     # 仅保留机器人可视化
```

#### 控制循环参数（v0.3.0 新增）

| 参数 | 说明 | 默认值 | 推荐范围 |
|------|------|--------|----------|
| `control_frequency` | 规划和控制循环频率（Hz） | `50.0` | `10.0 - 100.0` |

**调优指南：**
- **高速机器人**（>1 m/s）：50-100 Hz 以实现响应式控制
- **低速机器人**（<0.5 m/s）：20-30 Hz 即可，节省 CPU
- **嵌入式平台**：从 30 Hz 开始，根据需要增加

完整参数文档请参见 [config/robots/simulation/robot.yaml](config/robots/simulation/robot.yaml) 和 [config/robots/simulation/planner.yaml](config/robots/simulation/planner.yaml)。

---

## 📚 文档说明

### ROS2 话题

> **注意**：所有话题名称均可通过 ROS 参数进行配置。下表显示的是默认值。要自定义话题名称，请参见[话题配置参数](#话题配置参数新增)部分。

#### 订阅话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | 用于障碍物检测的激光扫描数据 |
| `/plan` | `nav_msgs/Path` | 全局路径航点 |
| `/clicked_point` | `geometry_msgs/PointStamped` | Publish Point 导航目标点 |

#### 发布话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/neupan_cmd_vel` | `geometry_msgs/Twist` | 速度命令（默认重映射到 `/cmd_vel`） |
| `/neupan_plan` | `nav_msgs/Path` | 优化后的轨迹 |
| `/neupan_ref_state` | `nav_msgs/Path` | 参考状态轨迹 |
| `/neupan_initial_path` | `nav_msgs/Path` | 初始路径可视化 |
| `/dune_point_markers` | `visualization_msgs/MarkerArray` | DUNE 网络可视化 |
| `/nrmp_point_markers` | `visualization_msgs/MarkerArray` | NRMP 网络可视化 |
| `/robot_marker` | `visualization_msgs/Marker` | 机器人轮廓可视化 |

### TF 坐标树

```
map
 └── odom（可选）
      └── base_link
           └── laser_link（如果独立）
```

### Launch 文件

| Launch 文件 | 用途 | 使用场景 |
|-------------|------|----------|
| `sim_complete.launch.py` | 完整仿真系统，包含 ddr_minimal_sim 和 NeuPAN | 仿真测试 |
| `simulation.launch.py` | 独立 NeuPAN 仿真配置，不启动 ddr_minimal_sim | 自定义集成与调试 |
| `limo.launch.py` | Limo 机器人部署 | 实体机器人导航 |
| `ranger.launch.py` | Ranger 机器人部署 | 实体机器人导航 |
| `scout.launch.py` | Scout 机器人部署 | 实体机器人导航 |

---

## 🏗️ 系统架构

### 线程安全

NeuPAN ROS2 采用线程安全的多线程架构，在多核系统上实现最佳性能：

**执行器：**
- **MultiThreadedExecutor**：支持并发回调处理，提升 CPU 利用率

**回调组：**
- **控制定时器**（`MutuallyExclusiveCallbackGroup`）：
  - 独立运行控制循环（`run()`）
  - 防止并发规划执行
  - 确保确定性的规划行为

- **传感器订阅**（`ReentrantCallbackGroup`）：
  - 扫描、路径和目标回调可并发运行
  - 优化多核系统上的传感器数据处理
  - 降低回调延迟

**状态保护：**
- 所有共享状态（`robot_state`、`obstacle_points`、规划器状态）均由 `threading.Lock` 保护
- 细粒度锁定最小化锁竞争（相比粗粒度锁定减少 75-95%）
- 规划期间并发传感器更新安全

**优势：**
- ✅ 多核系统上线程安全
- ✅ 传感器处理无竞态条件
- ✅ 最优 CPU 利用率
- ✅ 提升实时响应性

### 模块化设计

该包遵循模块化架构以提升可维护性：

**neupan_node.py**（主节点 ~800 行）：
- ROS2 集成层
- 订阅/发布管理
- 控制循环协调
- 将规划器集成到 ROS2 生态系统

**visualization_manager.py**（可视化模块 ~322 行）：
- 可选的 RViz 标记生成
- 独立于规划逻辑
- 线程安全的可视化发布
- 处理 DUNE、NRMP 和机器人轮廓标记
- 可在嵌入式平台上禁用以实现零开销

**utils.py**（工具模块 ~51 行）：
- 坐标转换辅助函数
- `yaw_to_quat()`：将偏航角转换为四元数
- `quat_to_yaw()`：从四元数提取偏航角
- 共享工具函数

**优势：**
- ✅ 清晰的关注点分离
- ✅ 易于维护和扩展
- ✅ 可选可视化降低 CPU 负载
- ✅ 可重用的工具函数

---

## 🔗 相关链接

- **原始 ROS1 封装**：[NeuPAN-ROS](https://github.com/hanruihua/neupan_ros)
- **核心算法库**：[NeuPAN](https://github.com/hanruihua/neupan)
- **研究论文**：[IEEE Transactions on Robotics (2025)](https://ieeexplore.ieee.org/document/10938329)
- **项目主页**：[NeuPAN Project Page](https://hanruihua.github.io/neupan_project/)
- **ROS2 Humble 文档**：[docs.ros.org/en/humble](https://docs.ros.org/en/humble/)

---

## 📄 开源协议

本项目采用 [GNU General Public License v3.0](LICENSE) 协议开源。

---

## 📖 引用

如果您觉得本代码或论文对您有帮助，感谢您为本仓库点个星标 ⭐ 并引用我们的论文：

```bibtex
@article{han2025neupan,
  title={Neupan: Direct point robot navigation with end-to-end model-based learning},
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Chen, Jianjun and Lin, Shijie and Li, Chengyang and Xu, Chengzhong and Eldar, Yonina C and Hao, Qi and others},
  journal={IEEE Transactions on Robotics},
  year={2025},
  publisher={IEEE}
}
```

---

## 🤝 致谢

- **NeuPAN 原始算法**：由香港大学 [Ruihua HAN](https://github.com/hanruihua) 及[SIAT-INVS](https://siat-invs.com/)团队开发
- **ROS2 集成**：针对 AgileX Limo 平台优化和测试
- **硬件合作伙伴**：AgileX x Hive Matrix（[sales@hive-matrix.com](mailto:sales@hive-matrix.com)）

---

## 📮 联系与支持

如有问题、反馈或合作机会，请联系：

- **Issues**：[GitHub Issues](https://github.com/KevinLADLee/neupan_ros2/issues)
- **此项目联系邮箱**：chengyangli@connect.hku.hk
- **原项目维护者**：hanrh@connect.hku.hk

---

**🎉 祝您使用 NeuPAN 导航愉快！🤖**
