# Module2：语义图与目标解析

> 公开版状态：静态图结构、Scout living-room 示例地图和查询逻辑保留；默认固定路线演示
> 不启动本模块。

Module2 应把 Module1 的结构化意图关联到场景实体、关系和目标位姿。它不负责 metric path、
局部避障或底盘控制。

## 接口和数据

- 实体、关系、位姿和拓扑查询 service
- 输入导航意图，输出带唯一实体 ID 和目标位姿的语义目标
- `maps/scout_kujiale_0021.yaml`：公开 living-room 静态语义示例
- `maps/DCT_hospital_demo.yaml`：历史回归数据，不是默认场景

图节点可表示 room、object、waypoint、door 和 corridor；拓扑可达不等于物理可通行，
后续 Module3 仍必须使用当前地图和 TF 验证。

## 正式替换与本机适配

替换场景时更新实体 ID、坐标系、地图原点和关系数据，并移除住户、设备序列号等敏感
字段。真实地图或数据库凭据放在仓库外。输出坐标必须明确 frame，并与 Module3/Isaac
使用的 `map`/`odom` 约定一致。

## 测试

```bash
cd "$SCOUT_WORKSPACE/llmModule/Module2"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_graph.py
```
