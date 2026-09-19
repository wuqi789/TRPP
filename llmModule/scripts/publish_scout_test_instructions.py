#!/usr/bin/env python3
"""List or publish the Scout semantic-navigation instruction corpus."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import yaml


DEFAULT_CASES = Path(__file__).with_name("scout_test_instructions.yaml")


def load_cases(path: Path) -> list[dict]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    cases = data.get("cases", [])
    if not isinstance(cases, list):
        raise ValueError("cases must be a list")
    identifiers = []
    for case in cases:
        if not isinstance(case, dict):
            raise ValueError("each case must be a mapping")
        for field in ("id", "category", "instruction", "expected"):
            if field not in case:
                raise ValueError(f"test case is missing '{field}'")
        identifiers.append(str(case["id"]))
    duplicates = sorted({value for value in identifiers if identifiers.count(value) > 1})
    if duplicates:
        raise ValueError(f"duplicate case ids: {duplicates}")
    return cases


def select_cases(cases: list[dict], ids: list[str], categories: list[str]) -> list[dict]:
    known_ids = {str(case["id"]) for case in cases}
    unknown = sorted(set(ids).difference(known_ids))
    if unknown:
        raise ValueError(f"unknown case ids: {unknown}")
    selected = cases
    if ids:
        requested = set(ids)
        selected = [case for case in selected if str(case["id"]) in requested]
    if categories:
        requested_categories = set(categories)
        selected = [
            case for case in selected if str(case["category"]) in requested_categories
        ]
    return selected


def print_case(index: int, total: int, case: dict) -> None:
    expected = case["expected"]
    result = (
        f"goal_id={expected.get('goal_id')}"
        if expected.get("module2_resolved")
        else f"error_code={expected.get('error_code')}"
    )
    print(
        f"[{index}/{total}] {case['id']} ({case['category']}): "
        f"{case['instruction']} -> {result}",
        flush=True,
    )


def publish(cases: list[dict], topic: str, interval: float, subscriber_timeout: float) -> None:
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("scout_instruction_batch_publisher")
    publisher = node.create_publisher(String, topic, 10)
    try:
        deadline = time.monotonic() + subscriber_timeout
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError(f"no Module1 subscriber discovered on {topic}")

        for index, case in enumerate(cases, start=1):
            print_case(index, len(cases), case)
            message = String()
            message.data = str(case["instruction"])
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.2)
            if index < len(cases):
                deadline = time.monotonic() + interval
                while time.monotonic() < deadline:
                    rclpy.spin_once(
                        node,
                        timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())),
                    )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
    parser.add_argument("--case", action="append", default=[], dest="case_ids")
    parser.add_argument(
        "--category",
        action="append",
        choices=("positive", "constraint", "negative"),
        default=[],
    )
    parser.add_argument("--topic", default="/user_instruction")
    parser.add_argument("--interval", type=float, default=35.0)
    parser.add_argument("--subscriber-timeout", type=float, default=10.0)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.interval < 0.0 or args.subscriber_timeout < 0.0:
        parser.error("timeouts must be non-negative")
    cases = select_cases(load_cases(args.cases), args.case_ids, args.category)
    if not cases:
        parser.error("no test cases selected")

    if args.list or args.dry_run:
        for index, case in enumerate(cases, start=1):
            print_case(index, len(cases), case)
        return 0

    publish(cases, args.topic, args.interval, args.subscriber_timeout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

