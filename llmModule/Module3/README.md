# Module3：导航可行性验证

> 公开版状态：实体、拓扑、几何和 Nav2 验证代码保留；默认固定路线演示不启动本模块。

Module3 应对已解析的目标执行 fail-closed 验证，不直接控制机器人：

1. 实体与地图版本一致；
2. 语义拓扑可达；
3. 目标和约束位姿在当前 costmap 中合法；
4. 隔离的 Nav2 planner 能产生全局可行结果。

主要输入/输出为 `/semantic_navigation/resolution`、
`/semantic_navigation/verification` 和
`/semantic_navigation/verification_readiness`。正式链路只有在 readiness 为真且所有启用
检查通过后才能交给 Module4。

## 正式替换与本机适配

部署者需要适配 map topic、global frame、robot base frame、footprint、Nav2 参数和 keepout
mask。`map -> odom -> base_link` 必须存在；不能用静态假 TF 掩盖里程计缺失。配置中的研究
开关仅用于隔离故障，任何 `SKIPPED/BYPASSED` 不能被报告为安全验收通过。

动态 LiDAR/相机安全属于执行层。静态规划成功不证明路径在执行时安全。

验证日志和 costmap 快照可能暴露室内结构，应写到仓库外的受控目录并按根
`SECURITY.md` 去敏。
