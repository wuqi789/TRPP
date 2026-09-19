# Module4：Waypoint 生成与执行监督

> 公开版状态：执行契约和参考实现保留；默认穿帘演示由 `fixed_route_driver` 替代动态路线
> provider，不访问云端模型。

正式 Module4 应把 Module3 已验证的目标转换为有序 waypoint，校验输出，并通过
`/clicked_point` 逐点驱动 NeuPAN。NeuPAN 仍负责局部运动与避障。

## 主要接口

| 方向 | 名称 | 类型 |
| --- | --- | --- |
| 输入 | `/semantic_navigation/verification` | `VerificationResult` |
| 输入 | `/semantic_validation/map` | `nav_msgs/OccupancyGrid` |
| 输入 | `/semantic_mapping/pushability_map` | `PushabilityMap` |
| Action | `/semantic_navigation/execute` | `ExecuteNavigation` |
| 输出 | `/semantic_navigation/status` | `NavigationTaskStatus` |
| 输出 | `/semantic_navigation/readiness` | `SystemReadiness` |
| 输出 | `/semantic_navigation/execution_route` | `nav_msgs/Path` |
| 输出 | `/clicked_point` | `PointStamped` |

路线 provider 的输出必须做 schema、边界、frame、free-space、连通性和任务关联校验。
可视化 polyline 不是碰撞安全证明；局部控制和执行 watchdog 仍不可省略。

## 公开 mock 与正式替换

公开版固定目标与 living-room 资产配套，并以 TF 距离和稳定时间判定成功。正式替换时保持
topic/action、QoS、任务去重、抢占、取消、hold goal、超时和终态语义。路线模型可以作为
独立 adapter 接入，不建议把私有网络调用写入 fixed route mock。

provider endpoint、模型、凭据、输出目录和地图参数都是本机适配项。路线图像、point
adjustment 和 execution audit 可能包含室内布局；默认不写入仓库，正式部署应保存到
权限受控且有到期策略的外部目录。

## 测试

```bash
cd "$SCOUT_WORKSPACE/llmModule"
source setup_scout_llm.sh
colcon test --packages-select llm_module4
colcon test-result --verbose
```

真实 provider 与 Isaac 结果必须单独报告；mock 成功只证明接口和状态机闭环。
