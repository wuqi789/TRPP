# Third-Party Components and Asset Notices

This file records provenance and release boundaries; it does not replace any third-party license.
Before publishing a GitHub release, maintainers must verify each proposed file against its terms;
files with unclear provenance or redistribution rights must not be published.

## Isaac Sim and InteriorAgent

Isaac Sim itself is not distributed in this repository. Users must obtain a compatible version and
point to the local installation through `ISAACSIM_PYTHON_EXE` or `ISAACSIM_PATH`.

The workspace currently contains the `isaacsim-assets/InteriorAgent/kujiale_0021` living-room
scene assets. These files come from the [InteriorAgent dataset](https://huggingface.co/datasets/spatialverse/InteriorAgent)
and are subject to the InteriorAgent Terms of Use linked by its directory README, not this project's
code license. Before publishing, maintainers must confirm that the terms permit the intended
distribution; if they cannot, remove the scene assets from the release and require users to
download them from the original source into the same relative path.

## NeuPAN and Checkpoints

`src/NeuPAN` and `src/neupan_ros2` retain their upstream GPL-3.0 licenses. The Scout checkpoint
required by the public demonstration is:

```text
src/neupan_ros2/src/neupan_ros2/config/robots/scout/models/dune_model_5000.pth
SHA-256: 66c55fedae14bc4684f0dee9c37fe5d0e5a86abfce74ddf99af2663e85edb8de
```

This file has the same checksum as the file at the same path on the `main` branch of upstream
`KevinLADLee/neupan_ros2`. Other robot checkpoints and training weights are outside this
demonstration and have been removed from the public workspace. If the model is replaced later,
record its source, license, checksum, and applicable robot dimensions again.

## Upstream Robot and Sensor Code

- `src/scout_ros2`: retains the Apache-2.0 license in its directory.
- `src/livox_ros_driver2`: retains the Livox/MIT third-party license notice.
- `src/ugv_sdk`, `scout_ros`, and `src/agx_arm_sim`: follow their upstream LICENSE files and
  dependency terms before release.
- `groundingdinoVLM/THIRD_PARTY_NOTICES.md`: records the adapter boundary for the public perception
  interface; deployers must verify dependencies, weights, and licenses for optional backends, which
  are not distributed with the default mock demonstration.

README files inside third-party directories are upstream component documentation and may describe
standalone examples, other robots, or networked installation methods. They do not represent this
project's public default startup chain. The root `README.md` defines the project's actual boundary
and runtime.
