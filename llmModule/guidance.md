# Module1-4 正式集成指南

> 当前公开默认演示不启动 Module1-4 的外部 provider。本文仅指导如何逐步接入正式实现，
> 不包含 endpoint、凭据、私有模型或生产数据。

## 接入顺序

1. 固定 Isaac 场景、`/map`、`/odom` 和 `map -> odom -> base_link` TF 契约。
2. 启用 Module2，用去敏静态语义图确认实体 ID、frame 和位姿。
3. 启用 Module3，等待所有 readiness 检查通过，验证实体、拓扑、几何和 planner。
4. 以离线 mock 启用 Module1，验证自然语言到 canonical intent 的 schema。
5. 以本地固定路线 provider 启用 Module4，验证抢占、取消、hold、FIFO 和终态。
6. 最后分别替换 Module1/Module4 provider；每次只改变一个网络边界。

## 必须保持的约束

- 所有节点使用一致的 ROS domain、RMW 和仿真时钟。
- Module3 未通过时 Module4 不得发布运动目标。
- waypoint 必须校验 frame、边界、可通行点和任务关联。
- 模型路线不是碰撞安全证明，NeuPAN 和执行 watchdog 仍必须工作。
- provider 超时、非法响应、TF 缺失、重复节点或状态不确定时失败关闭。
- 正式包保持 `semantic_navigation_interfaces` 的消息/action 兼容。

## 本机适配

| 项目 | 需要确认的内容 |
| --- | --- |
| Isaac | 本机 Python、ROS bridge、场景资产和出生位姿 |
| Module2 | 地图 revision、实体 ID、坐标系和目标位姿 |
| Module3 | map topic、costmap、footprint、TF 和 Nav2 参数 |
| Module1/4 | 外部 adapter、模型名、endpoint、CA、超时和 schema |
| 运行数据 | 仓库外受控目录、权限、保留期和脱敏流程 |

凭据只通过部署平台 secret manager 注入。不要在命令示例中赋值或回显；缺失时应立即
失败。诊断图、路线摘要和执行记录默认写入 `$SCOUT_WORKSPACE/runtime/`（已忽略），
provider 原始响应、Prompt 和 traceback 不会被公开代码持久化；生产部署应通过配置改到
仓库外。

## 验证

先运行各模块离线测试，再在隔离 ROS domain 中逐层观察 intent、resolution、verification、
status 和 `/clicked_point`。正式结果必须区分离线 mock、Isaac 集成和实体硬件三种证据，
不能用 mock 成功替代真实 provider 或机械安全验收。
