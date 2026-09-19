# Scout Mini core-framework data-flow figure

> This figure describes the intended full system. In the public repository's
> default launch, private perception/reasoning providers and arm motion are
> replaced by deterministic local mocks; Isaac Sim, TF, LiDAR, NeuPAN, and base
> motion remain executable.

The figure source is `render_figure.py`. Rendered deliverables are generated as SVG, PDF,
and 300-dpi PNG so that the vector version can be used in the manuscript and the raster version can
be used for review.

## Figure caption (English)

**Data flow of the proposed LLM-guided semantic navigation and event-triggered obstacle traversal
framework.** A natural-language instruction is converted into a constrained semantic intent,
grounded against a static scene graph, and verified by deterministic entity, topology, geometry,
and planner checks. Only a verified request reaches the Module 4 execution bridge, where
model-proposed coarse waypoints are sanitized and dispatched through a FIFO interface to NeuPAN.
During execution, a synchronized LiDAR--RGB event coordinator identifies obstacles intersecting the
current path corridor and invokes two independent provider-neutral gates in parallel: target
verification and pushability assessment. A tracked obstacle is granted a time-bounded
scan-filtering and low-speed traversal authorization only when both gates pass;
rejection, timeout, provider failure, or watchdog expiry retains/restores the full scan and yields a
conservative avoidance or stop response.

## 图注（中文）

**本文所提出的语义导航与事件触发式障碍物通行框架的数据流。** 自然语言指令首先被转换为结构化语义意图，随后在静态场景图上完成实体落地，并通过确定性验证。只有验证通过的请求才能进入 Module 4 执行桥；该模块将路线候选清洗后通过 FIFO 接口提交给 NeuPAN。执行过程中，同步的 LiDAR--RGB 事件协调器检测当前路径走廊内的障碍物，并并行调用目标验证和可推性评估两条门控链。仅当两条门控均通过时，系统才对被跟踪障碍物授予有时限的激光簇过滤和低速通行权限；拒绝、超时、服务失败或看门狗触发时，系统保留或恢复完整激光扫描，并采用保守绕行或停车策略。

## Rendering

```bash
python3 render_figure.py
```

The diagram describes the formal authorization architecture and does not claim
that the public mock performs physical pushing. Public-demo acceptance is limited
to actual Scout motion through the living-room curtain route in Isaac Sim.
