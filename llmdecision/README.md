# 障碍物可推性判别模块

> 公开版状态：输入/输出 schema、ROS action、provider 抽象和离线 mock 测试保留；默认
> 穿帘演示使用 `scout_public_mock`，不启动本目录的外部 LLM/VLM provider。

## 正式职责

本模块应融合结构化场景描述、车辆状态、机械交互状态和可选知识库，返回障碍类别、
`pushability_probability`、`push/avoid/stop` 建议、不确定性和风险标记。输出是决策证据，
不是速度、机械臂或过滤授权命令。

ROS 集成通过 `/llmdecision/assess_pushability` action 暴露。公开 mock 保持同一 action 类型
并固定返回通过，仅用于验证跨模块数据流。

## 目录边界

- `api/`：严格输入和输出 schema
- `core/`：决策、融合、置信度和结果存储
- `llm/`：provider 抽象及离线 mock
- `vlm/`：视觉描述 provider 抽象
- `rag/`：本地示例知识库
- `llmdecision_ros/`：ROS 2 action adapter
- `config/`：无凭据的示例配置
- `examples/`：仅保留去敏 JSON schema 夹具，不保存真实图片或运行记录

## 正式替换与本机适配

正式 provider 可在 `llm/`、`vlm/` 中实现，也可以由独立私有包提供 adapter。必须保持
schema、action 名称、`request_id`、取消、超时和保守失败语义。公开代码只读取
`SCOUT_DECISION_ADAPTER`、`SCOUT_PERCEPTION_ADAPTER` 的 `module:factory` 值；适配器自行
管理模型、传输、凭据、证书、重试和私有路径。

不要在 README、YAML、shell 参数或命令历史中写凭据。使用部署平台的 secret manager
向当前进程注入，且错误日志不得包含 header、响应正文、远端 stderr 或环境变量值。

## 运行数据

默认运行产物写入 `llmdecision/runtime/`：

- `latest.json`：最近一次完成记录
- `history/`：请求历史
- `batch_*`、`latency_*`：批处理和延迟测试输出

`runtime/` 已被 Git 忽略，结果文件使用受限文件权限。公开代码不会保存 provider 原始
响应、Prompt、traceback 或内部路径；真实图像和场景描述仍可能包含敏感信息，生产部署应
把路径覆盖到仓库外的加密/受控存储并设置到期清理。
不要把运行输出复制回 `examples/`。

批处理默认查找 `examples/images/`，但公开仓库故意不提供该目录中的图片。测试者应在本机
放入自行拥有且已去敏的 JPEG/PNG，或通过 `--images-dir` 指向仓库外的受控测试集；该目录
已被 `.gitignore` 排除。

## 本地测试

离线测试不需要网络或凭据：

```bash
cd "$SCOUT_WORKSPACE/llmdecision"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests
```

独立正式运行不是公开演示的组成部分。启用外部 provider 前，先阅读根 `SECURITY.md`，
并确认所有配置仍使用外部 secret 注入和仓库外运行目录。
