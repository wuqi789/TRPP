# Module1：语言意图解析

> 公开版状态：接口、JSON 校验、provider 抽象和离线测试保留；默认穿帘演示不启动本模块。

Module1 应把 `/user_instruction` 的自然语言请求转换为结构化导航意图。它只负责语言边界，
不生成速度、轨迹或控制命令。

## 接口

- 输入：`/user_instruction` (`std_msgs/msg/String`)
- 输出：`/navigation_intent` 或集成链中的等价 canonical intent
- 结果字段：目标、参考实体、空间关系、约束和策略

模型输出必须经过严格结构校验；超时、鉴权失败、非法 JSON 或未知字段均不得伪造成功。

## 正式替换与本机适配

`llm_agent/llm_interface.py` 是 provider 边界。正式实现可以接本地模型或外部服务，但需
保持 parser 契约和失败语义。endpoint、模型名、CA、代理和凭据由部署环境提供；公开配置
只保留无效示例 endpoint 与环境变量名。不要把真实值写入 `config/*.yaml`。

## 测试

```bash
cd "$SCOUT_WORKSPACE/llmModule/Module1"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_parser.py
```

离线测试使用明确的占位值，不访问网络，也不得打印 credential 环境变量。
