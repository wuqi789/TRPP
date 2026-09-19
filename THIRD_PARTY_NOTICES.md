# 第三方组件与资产说明

本文只记录来源和发布边界，不替代任何第三方许可证。发布 GitHub 版本前，维护者必须逐项
确认拟发布文件符合对应条款；来源或再分发权限不明确的文件不得发布。

## Isaac Sim 与 InteriorAgent

Isaac Sim 本体不在本仓库中分发。使用者须自行取得兼容版本，并通过
`ISAACSIM_PYTHON_EXE` 或 `ISAACSIM_PATH` 指向本机安装。

工作区当前包含 `isaacsim-assets/InteriorAgent/kujiale_0021` living-room 场景资产。这些
文件来源于 [InteriorAgent 数据集](https://huggingface.co/datasets/spatialverse/InteriorAgent)，
适用其目录 README 所链接的 InteriorAgent Terms of Use，而不是本项目代码许可证。维护者
在公开仓库前必须确认这些条款允许目标分发方式；若不能确认，应从发行包删除场景资产，
并要求使用者按原始来源自行下载到同一相对路径。

## NeuPAN 与 checkpoint

`src/NeuPAN` 和 `src/neupan_ros2` 保留各自的上游 GPL-3.0 许可证。公开演示运行所需的
Scout checkpoint 为：

```text
src/neupan_ros2/src/neupan_ros2/config/robots/scout/models/dune_model_5000.pth
SHA-256: 66c55fedae14bc4684f0dee9c37fe5d0e5a86abfce74ddf99af2663e85edb8de
```

该文件与 `KevinLADLee/neupan_ros2` 上游 `main` 分支中的同路径文件校验值一致。其他机器人
checkpoint 和训练过程权重不属于本演示，已从公开工作区移除。若后续替换模型，必须重新
记录来源、许可证、校验值和适用机器人尺寸。

## 机器人与传感器上游代码

- `src/scout_ros2`：保留目录中的 Apache-2.0 许可证。
- `src/livox_ros_driver2`：保留目录中的 Livox/MIT 第三方许可证说明。
- `src/ugv_sdk`、`scout_ros`、`src/agx_arm_sim`：发布前以各自上游 LICENSE 和依赖条款为准。
- `groundingdinoVLM/THIRD_PARTY_NOTICES.md`：记录公开感知接口的适配边界；可选后端的
  依赖、权重和许可证由部署者自行核验，不随默认 mock 演示分发。

第三方目录内的 README 是上游组件文档，可能描述独立示例、其他机器人或联网安装方式，
不代表本项目公开默认启动链。项目实际边界和运行方式以根 `README.md` 为准。
