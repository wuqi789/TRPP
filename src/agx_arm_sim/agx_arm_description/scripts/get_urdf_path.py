#!/usr/bin/env python3
"""
agx_arm_description — scripts/get_urdf_path.py
================================================
查询指定机械臂型号与末端执行器组合对应的 URDF/Xacro 文件路径。
常用于 CLI 调试、脚本集成或在 launch 文件外部获取 robot_description。

用法:
  # 查询 piper 带夹爪的 URDF 文件路径
  ros2 run agx_arm_description get_urdf_path.py --arm_type piper --end_effector gripper

  # 打印解析后的 URDF XML（适合管道传递）
  ros2 run agx_arm_description get_urdf_path.py --arm_type nero --print_xml

  # 列出所有支持的型号和末端执行器
  ros2 run agx_arm_description get_urdf_path.py --list
"""

import argparse
import os
import sys

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    print("[ERROR] ament_index_python 未找到，请在 ROS2 环境中运行此脚本。", file=sys.stderr)
    sys.exit(1)

try:
    import xacro
except ImportError:
    xacro = None

# 与 display.launch.py 保持同步
ARM_URDF_MAP = {
    "piper": {
        "none":        "piper_description.urdf",
        "gripper":     "piper_with_gripper_description.xacro",
        "revo2_left":  "piper_with_left_revo2_description.xacro",
        "revo2_right": "piper_with_right_revo2_description.xacro",
    },
    "piper_h": {
        "none":        "piper_h_description.urdf",
        "gripper":     "piper_h_with_gripper_description.xacro",
        "revo2_left":  "piper_h_with_left_revo2_description.xacro",
        "revo2_right": "piper_h_with_right_revo2_description.xacro",
    },
    "piper_l": {
        "none":        "piper_l_description.urdf",
        "gripper":     "piper_l_with_gripper_description.xacro",
        "revo2_left":  "piper_l_with_left_revo2_description.xacro",
        "revo2_right": "piper_l_with_right_revo2_description.xacro",
    },
    "piper_x": {
        "none":        "piper_x_description.urdf",
        "gripper":     "piper_x_with_gripper_description.xacro",
        "revo2_left":  "piper_x_with_left_revo2_description.xacro",
        "revo2_right": "piper_x_with_right_revo2_description.xacro",
    },
    "nero": {
        "none":        "nero_description.urdf",
        "gripper":     "nero_with_gripper_description.xacro",
        "revo2_left":  "nero_with_left_revo2_description.xacro",
        "revo2_right": "nero_with_right_revo2_description.xacro",
    },
    "revo2": {
        "left":  "revo2_left_hand.urdf",
        "right": "revo2_right_hand.urdf",
    },
}


def get_urdf_filepath(arm_type: str, end_effector: str, revo2_side: str) -> str:
    pkg_share = get_package_share_directory("agx_arm_description")
    urdf_base = os.path.join(pkg_share, "agx_arm_urdf")

    type_map = ARM_URDF_MAP.get(arm_type)
    if type_map is None:
        raise ValueError(f"不支持的机械臂型号: {arm_type}")

    if arm_type == "revo2":
        key = revo2_side if revo2_side in type_map else "right"
    else:
        key = end_effector if end_effector in type_map else "none"

    filename = type_map[key]
    return os.path.join(urdf_base, arm_type, "urdf", filename)


def main():
    parser = argparse.ArgumentParser(
        description="查询 AgileX 机械臂 URDF 文件路径"
    )
    parser.add_argument(
        "--arm_type", default="piper",
        choices=list(ARM_URDF_MAP.keys()),
        help="机械臂型号 (默认: piper)"
    )
    parser.add_argument(
        "--end_effector", default="none",
        choices=["none", "gripper", "revo2_left", "revo2_right"],
        help="末端执行器 (默认: none)"
    )
    parser.add_argument(
        "--revo2_side", default="right",
        choices=["left", "right"],
        help="revo2 左/右手 (默认: right)"
    )
    parser.add_argument(
        "--print_xml", action="store_true",
        help="打印解析后的 URDF XML 内容"
    )
    parser.add_argument(
        "--list", action="store_true",
        help="列出所有支持的型号和可选配置"
    )

    args = parser.parse_args()

    if args.list:
        print("\n支持的机械臂型号及可选末端执行器:\n")
        for arm, variants in ARM_URDF_MAP.items():
            print(f"  arm_type: {arm}")
            for key, fname in variants.items():
                print(f"    └─ {key:12s} -> {fname}")
        print()
        return

    try:
        filepath = get_urdf_filepath(args.arm_type, args.end_effector, args.revo2_side)
    except ValueError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)

    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}", file=sys.stderr)
        print("请确认已执行: git submodule update --init --recursive", file=sys.stderr)
        sys.exit(1)

    if args.print_xml:
        if filepath.endswith(".xacro"):
            if xacro is None:
                print("[ERROR] xacro 模块未安装", file=sys.stderr)
                sys.exit(1)
            doc = xacro.process_file(filepath)
            print(doc.toxml())
        else:
            with open(filepath, "r") as f:
                print(f.read())
    else:
        print(filepath)


if __name__ == "__main__":
    main()
