# agx_arm_sim README

AgileX 系列机械臂的 ROS2 仿真与开发工具集，为机械臂的算法开发、仿真测试、可视化调试提供完整的功能包支持，整合了机械臂描述、运动规划、仿真适配等能力，帮助开发者快速搭建机械臂的开发环境，支持从模型可视化到运动规划、物理仿真的全流程开发。

## 仓库结构

本仓库整合了多个子功能包，为机械臂开发提供完整的能力支持，整体结构如下：

```Plain
agx_arm_sim/
├── agx_arm_description/   # 机械臂 URDF/Xacro 描述功能包
│   ├── 提供参数化的机械臂模型，支持多种型号与末端执行器的灵活配置
│   ├── 包含 USD 格式模型，可直接用于 Isaac Sim 等仿真环境
│   └── 支持 RViz2 可视化调试，快速验证模型配置
├── Moveit2/               # MoveIt2 运动规划功能包
│   ├── 提供运动学求解、碰撞检测、轨迹规划能力
│   └── 适配全系列机械臂型号，支持运动规划的仿真与测试
├── realsense2_description/ # RealSense 相机描述功能包
│   └── 提供 RealSense D435 等相机的 URDF 模型，支持相机的仿真与可视化
└── .gitmodules            # 子模块配置文件，用于管理模型资源子模块
```

## 下载与安装

### 1. 克隆仓库并初始化子模块

```bash
cd ~/ros2_ws/src
# 克隆本仓库
git clone https://github.com/agilexrobotics/agx_arm_sim.git
# 初始化子模块，拉取完整的模型资源
cd agx_arm_sim
git submodule update --init --recursive
```

### 2. 安装依赖

```bash
# 安装所需的 ROS2 依赖包
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs ros-humble-topic-based-ros2-control ros-humble-moveit*
```

### 3. 编译工作空间

```bash
colcon build
# 配置环境变量
source install/setup.bash
```

## 支持的机械臂型号

本仓库完整支持 AgileX 全系列机械臂，可通过参数灵活切换型号与末端配置，具体支持的型号如下：

| 型号标识  | 型号名称     | 支持的末端执行器                                       |
| --------- | ------------ | ------------------------------------------------------ |
| `piper`   | Piper        | 无末端 / 电动夹爪 / Revo2 灵巧手 / 示教器 / Pika2 夹爪 |
| `piper_h` | Piper H      | 无末端 / 电动夹爪 / Revo2 灵巧手 / 示教器 / Pika2 夹爪 |
| `piper_l` | Piper L      | 无末端 / 电动夹爪 / Revo2 灵巧手 / 示教器 / Pika2 夹爪 |
| `piper_x` | Piper X      | 无末端 / 电动夹爪 / Revo2 灵巧手 / 示教器 / Pika2 夹爪 |
| `nero`    | Nero         | 无末端 / 电动夹爪 / Revo2 灵巧手 / 示教器 / Pika2 夹爪 |
| `revo2`   | Revo2 灵巧手 | 支持左右手独立配置                                     |

## 支持的仿真环境

本仓库提供了多类仿真与开发环境的适配，满足不同的开发测试需求，当前支持的环境如下：

| 仿真环境             | 支持说明                                                     |
| -------------------- | ------------------------------------------------------------ |
| Isaac Sim 高保真仿真 | 提供完整的 USD 格式机械臂模型，可直接导入 NVIDIA Isaac Sim 环境，支持高精度的物理仿真，适配具身智能相关的开发需求 |
| MoveIt2 规划仿真     | 通过 MoveIt2 功能包，可完成机械臂的运动规划仿真测试，支持碰撞检测、轨迹规划的验证，快速开发机械臂的控制算法 |
| RViz2 可视化调试     | 支持在 RViz2 中完成机械臂的模型可视化、关节调试，无需启动完整仿真即可快速验证机械臂模型配置的正确性 |

| 机械臂型号 | Isaac Sim | Moveit2 | Mujuco | Gazebo |
| ---------- | --------- | ------- | ------ | ------ |
| `piper`    | ✅         | ✅       | ✅      | ✅      |
| `piper_h`  | ✅         | ✅       |        | ✅      |
| `piper_l`  | ✅         | ✅       |        | ✅      |
| `piper_x`  | ✅         | ✅       |        | ✅      |
| `nero`     | ✅         | ✅       | ✅      | ✅      |

## License

MIT License
