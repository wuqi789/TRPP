#!/usr/bin/env python3
"""Interactive terminal test for the Module1 semantic reasoner."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from llm_agent.json_parser import NavigationIntentParseError, parse_navigation_intent
from llm_agent.llm_interface import LLMError, LLMInterface
from llm_agent.utils import load_yaml


DEFAULT_CONFIG = MODULE_ROOT / "config" / "config.yaml"
def reason(instruction: str, llm: LLMInterface) -> dict:
    """Run the same LLM and parser layers used by the ROS 2 node."""
    response = llm.infer(instruction)
    return parse_navigation_intent(response).to_dict()


def print_result(instruction: str, llm: LLMInterface) -> bool:
    try:
        intent = reason(instruction, llm)
    except (LLMError, NavigationIntentParseError, ValueError) as exc:
        print(f"\n[ERROR] 解析失败：{exc}\n", file=sys.stderr)
        return False

    print("\nNavigation Intent:")
    print(json.dumps(intent, ensure_ascii=False, indent=2))
    print()
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactively test Module1 navigation intent parsing")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="LLM config YAML path")
    parser.add_argument("--once", metavar="INSTRUCTION", help="parse one instruction and exit")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        llm = LLMInterface(load_yaml(args.config))
    except (OSError, ValueError) as exc:
        print(f"[ERROR] 初始化失败：{exc}", file=sys.stderr)
        return 2

    print(f"Module1 交互测试已启动（LLM mode: {llm.mode}）")
    if args.once is not None:
        return 0 if print_result(args.once.strip(), llm) else 1

    print("请输入自然语言导航指令；输入 quit 或 exit 退出。\n")
    while True:
        try:
            instruction = input("instruction> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n已退出。")
            return 0

        if instruction.casefold() in {"quit", "exit", "q"}:
            print("已退出。")
            return 0
        if not instruction:
            continue
        print_result(instruction, llm)


if __name__ == "__main__":
    raise SystemExit(main())
