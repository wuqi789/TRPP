# 正式后端验收边界

> 版本状态：外部适配。本文定义接口和安全要求，不表示公开仓库包含私有模型、云端
> provider、SSH 推理流程、模型权重或真实机械臂控制实现。

公开版本的唯一开箱验收是 Isaac living-room 固定路线穿帘，使用：

```bash
./run_traversal_system.sh headless:=true use_rviz:=false
```

正式后端入口 `SCOUT_PUBLIC_BACKEND=real` 仅供私有 adapter 集成。它会在启动前检查所需
环境变量，缺失时立即失败，绝不通过终端提示读取凭据。正式验收还要求部署方自行提供：

- 许可合规的感知与语义模型及其运行时资产；
- 云端或本地 VLM/LLM adapter；
- 已验证 host key 的远端执行环境（若使用 SSH）；
- Piper 驱动、限力、急停、碰撞保护和复位策略；
- 数据保留、访问控制、日志脱敏与凭据轮换方案。

## 保留的契约

正式 adapter 必须保持下列 action：

- `/groundingdino_vlm/verify_target`
- `/llmdecision/assess_pushability`
- `/piper/probe_pushability`
- `/obstacle_traversal/approach_obstacle`

还必须保持 readiness/status、`/piper/yolo_*`、`/isaac_joint_states`、
`/isaac_joint_command` 和 traversal 授权话题的类型、QoS、超时及失败语义。

## 验收数据

`run_real_traversal_acceptance.sh` 会产生 rosbag、JSONL 和日志，可能包含图像、场景结构、
位姿、模型响应及本机信息。默认输出目录已被 Git 忽略。正式部署应把输出重定向到仓库外
的权限受控目录，并设置自动到期清理。发布结果时只保留汇总指标和去敏证据。

脚本要求显式设置 `SCOUT_ALLOW_SENSITIVE_RECORDING=1`，否则拒绝启动录制。建议同时把
第二个参数设为仓库外的受控目录；该确认开关不等于数据已经脱敏。

## 判定原则

真实后端的验收不能用公开 mock 结果替代。至少分别证明：感知输入来自当前相机帧，
各 action 的 `request_id` 一致，机械探测完成并安全复位，过滤授权仅作用于目标扫描簇，
拒绝时保持完整障碍物，最终 Isaac/实体里程计实际越过目标。任何缺失 provider、超时、
解析失败或机械臂状态不确定都必须失败关闭。

正式模式的环境变量名称属于接口，可在源码中保留；变量值不得进入仓库、命令行参数、
日志、rosbag 或验收报告。完整规则见 `SECURITY.md`。
