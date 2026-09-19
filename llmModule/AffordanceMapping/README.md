# 可推性语义映射

> 公开版状态：消息、融合边界和参考节点保留；默认穿帘演示不启动外部 VLM，可推性由本地
> mock action 固定返回。

正式 `llm_semantic_affordance` 应从稀疏 RGB-D 关键帧、语义类别和 TF 生成会话级可推性
实例，发布 `/semantic_mapping/pushability_map`、markers、overlay 和 readiness。它只提供
规划元数据，不授权机械推动，也不替代碰撞检测。

正式 provider 必须校验结构化结果、关联当前帧和 frame，并在图像、深度、TF、网络或解析
失败时保持保守结果。相机帧、模型响应和实例快照默认不应落盘；确需诊断时写到仓库外的
权限受控目录。

本机适配项包括相机 topic、标定、TF、关键帧阈值、模型 endpoint 和凭据。凭据只由部署
环境注入，不能出现在配置、日志或模型请求归档中。
