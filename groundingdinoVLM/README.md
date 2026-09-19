# 目标验证与障碍穿越接口

> 公开版状态：ROS 2 消息、action、穿越控制和安全闸保留；未随仓库发布的感知后端和
> 真实 Piper 探测不进入默认链路，由 `scout_public_mock` 替代。

## 模块职责

- `groundingdino_vlm`：应从同步图像和目标区域产生目标身份、置信度与验证证据。
- `piper_probe`：应进行障碍分类、受控机械探测和安全复位。
- `obstacle_traversal`：负责接近障碍、融合多模态结果、授权局部扫描过滤并限制速度。
- `scout_public_mock`：公开演示的确定性 provider、静态机械臂接口和固定路线驱动。
- `*_interfaces`：跨包使用的消息、service 和 action 契约。

公开默认入口不会加载模型权重、访问网络或读取凭据。它保留：

- `/groundingdino_vlm/verify_target`
- `/llmdecision/assess_pushability`
- `/piper/probe_pushability`
- `/obstacle_traversal/approach_obstacle`
- `/piper/yolo_ready`、`/piper/yolo_obstacle_result`
- `/isaac_joint_states`、`/isaac_joint_command`

mock 机械臂持续发布静态关节状态，接收关节命令但不执行运动。mock 结果只能验证接口、
状态机和导航闭环，不能作为感知准确率或机械安全证据。

## 构建与运行

从仓库根目录按根 `README.md` 构建，然后：

```bash
export ISAACSIM_PYTHON_EXE=/path/to/isaacsim/python.sh
./run_traversal_system.sh
```

默认 `SCOUT_PUBLIC_BACKEND=mock`。正式模式只为外部 adapter 保留入口；公开仓库不包含
私有实现。部署方应从 secret manager 向进程环境注入所需值，不要在文档、YAML、命令行
参数或 shell 历史中填写凭据。缺少配置时脚本会立即失败且不会交互等待。

## 正式替换要求

可以在各自包内实现正式 provider，但不要修改下游接口来绕过契约。替换时保持 action
名称、消息类型、QoS、`request_id` 关联、超时、取消和失败关闭行为；模型路径、endpoint、
设备编号和硬件参数应放入未提交的本机配置。私有算法也可以作为独立 ROS 包覆盖同一接口。

## 测试

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  groundingdinoVLM/src/scout_public_mock/test \
  groundingdinoVLM/src/obstacle_traversal/test/test_four_terminal_contract.py
```

测试占位值不是凭据，测试也不得打印环境变量内容。运行日志和图像按根 `SECURITY.md` 处理。
