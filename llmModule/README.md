# 语义导航模块

> 公开版状态：Module1-4 的契约、静态语义数据和参考实现保留，但默认穿帘演示不启动
> 外部语义 provider。`scout_public_mock/fixed_route_driver` 直接提供确定性 waypoint 和执行状态。

## 正式职责

```text
/user_instruction
  -> Module1: 语言意图
  -> Module2: 语义图实体与位姿解析
  -> Module3: 实体/拓扑/几何/全局规划验证
  -> Module4: waypoint 提议、校验和队列执行
  -> /clicked_point -> NeuPAN
```

`Interfaces` 定义共享 ROS 类型；`AffordanceMapping` 是可选可推性语义层。Module1 和
Module4 的外部推理属于正式 adapter，不是公开默认能力。

| 目录 | 当前公开内容 | 默认演示 |
| --- | --- | --- |
| `Interfaces` | 消息和 action 契约 | 使用 |
| `Module1` | parser、provider 边界、mock 单测 | 不启动 |
| `Module2` | 静态语义图和查询 | 不启动 |
| `Module3` | 验证逻辑和隔离 Nav2 配置 | 不启动 |
| `Module4` | 路线 provider 边界和执行监督 | 由固定路线替代 |
| `AffordanceMapping` | 可推性地图接口 | 不启动 |

## 正式接入

正式部署可以直接在模块目录中实现，但要保持 `semantic_navigation_interfaces`、topic/action
名称、QoS、任务去重、取消和终态语义。推荐按 Module1 到 Module4 逐个替换，并先使用
离线、去敏数据完成契约测试。

模型 endpoint、API 凭据、私有地图和诊断目录都是本机适配项。凭据只由 secret manager
注入当前进程，不写入 YAML 或命令行。路线图像、模型响应和 execution audit 可能包含
场景信息，应写到仓库外的受控运行目录。

## 构建与测试

依赖准备脚本可能联网并创建 `.venv`；这些内容不提交。示例：

```bash
cd "$SCOUT_WORKSPACE/llmModule"
source /opt/ros/humble/setup.bash
./scripts/prepare_scout_dependencies.sh
source setup_scout_llm.sh
colcon build --symlink-install --packages-select \
  semantic_navigation_interfaces semantic_navigation_adapters \
  llm_semantic_affordance llm_module1 llm_module2 llm_module3 llm_module4
```

公开固定路线演示仍从仓库根目录运行 `run_traversal_system.sh`，不需要启动此处的云端
provider。各模块的边界和本机适配项见对应 README。
