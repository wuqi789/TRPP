#!/usr/bin/env python3
"""Exercise the live cloud -> semantic map -> Nav2 verification ROS chain."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from semantic_navigation_interfaces.msg import (
    NavigationIntent,
    ResolutionResult,
    SystemReadiness,
    VerificationResult,
)


CASES = (
    {
        "name": "english_sofa",
        "instruction": "Navigate to the sofa",
        "goal_object": "sofa",
        "goal_id": "living_room_sofa_0001",
        "verified": True,
    },
    {
        "name": "chinese_sofa",
        "instruction": "前往沙发",
        "goal_object": "sofa",
        "goal_id": "living_room_sofa_0001",
        "verified": True,
    },
    {
        "name": "avoid_sink",
        "instruction": "Navigate to the sofa while avoiding the sink",
        "goal_object": "sofa",
        "goal_id": "living_room_sofa_0001",
        "constraint": "avoid sink",
        "verified": True,
    },
    {
        "name": "via_entrance",
        "instruction": "Navigate to the sofa via the apartment entrance",
        "goal_object": "sofa",
        "goal_id": "living_room_sofa_0001",
        "constraint": "via apartment entrance",
        "verified": True,
    },
    {
        "name": "blocked_refrigerator",
        "instruction": "Navigate to the refrigerator",
        "goal_object": "refrigerator",
        "goal_id": "kitchen_fridge_0000",
        "verified": False,
    },
)


class LiveObserver(Node):
    def __init__(self) -> None:
        super().__init__("module1_to_module3_live_validation")
        self.readiness = None
        self.intents = {}
        self.resolutions = {}
        self.verifications = {}
        self.publisher = self.create_publisher(String, "/user_instruction", 10)
        self.create_subscription(
            SystemReadiness,
            "/semantic_navigation/verification_readiness",
            self._on_readiness,
            10,
        )
        self.create_subscription(
            NavigationIntent,
            "/semantic_navigation/intent",
            self._on_intent,
            10,
        )
        self.create_subscription(
            ResolutionResult,
            "/semantic_navigation/resolution",
            self._on_resolution,
            10,
        )
        self.create_subscription(
            VerificationResult,
            "/semantic_navigation/verification",
            self._on_verification,
            10,
        )

    def _on_readiness(self, message) -> None:
        self.readiness = message

    def _on_intent(self, message) -> None:
        self.intents[message.source_text] = message

    def _on_resolution(self, message) -> None:
        self.resolutions[message.request_id] = message

    def _on_verification(self, message) -> None:
        self.verifications[message.request_id] = message


def wait_for(executor, predicate, timeout: float, description: str):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        result = predicate()
        if result is not None and result is not False:
            return result
    raise TimeoutError(f"Timed out waiting for {description}")


def check_rows(message) -> list[dict[str, str]]:
    return [
        {
            "name": value.name,
            "status": value.status,
            "code": value.code,
            "message": value.message,
        }
        for value in message.checks
    ]


def node_subscribes(node: LiveObserver, topic: str, expected_node: str) -> bool:
    return any(
        endpoint.node_name == expected_node
        for endpoint in node.get_subscriptions_info_by_topic(topic)
    )


def normalized_constraint(value: str) -> str:
    return " ".join(value.casefold().replace("_", " ").split())


def run_case(node, executor, case, timeout: float) -> dict:
    instruction = case["instruction"]
    node.intents.pop(instruction, None)
    request = String()
    request.data = instruction
    node.publisher.publish(request)

    intent = wait_for(
        executor,
        lambda: node.intents.get(instruction),
        timeout,
        f"Module1 intent for {case['name']}",
    )
    resolution = wait_for(
        executor,
        lambda: node.resolutions.get(intent.request_id),
        timeout,
        f"Module2 resolution for {case['name']}",
    )
    verification = wait_for(
        executor,
        lambda: node.verifications.get(intent.request_id),
        timeout,
        f"Module3 verification for {case['name']}",
    )

    assert intent.goal_object == case["goal_object"]
    if "constraint" in case:
        expected = normalized_constraint(case["constraint"])
        assert expected in {
            normalized_constraint(value) for value in intent.constraints
        }
    assert resolution.resolved is True
    assert resolution.goal_id == case["goal_id"]
    assert verification.verified is case["verified"]
    checks = {value.name: value for value in verification.checks}
    assert checks["entity"].status == "PASS"
    assert checks["topology"].status == "PASS"
    if case["verified"]:
        assert [checks[name].status for name in ("geometry", "planner")] == [
            "PASS",
            "PASS",
        ]
        assert verification.planning_length > 0.0
        assert verification.planning_time > 0.0
    else:
        assert any(
            checks[name].status == "FAIL" for name in ("geometry", "planner")
        )

    return {
        "case": case["name"],
        "status": "PASS",
        "instruction": instruction,
        "module1": {
            "request_id": intent.request_id,
            "goal_object": intent.goal_object,
            "reference_object": intent.reference_object,
            "relation": intent.relation,
            "constraints": list(intent.constraints),
            "strategy": intent.strategy,
        },
        "module2": {
            "resolved": resolution.resolved,
            "goal_id": resolution.goal_id,
            "canonical_name": resolution.canonical_name,
            "map_revision": resolution.map_revision,
            "goal_pose": [
                resolution.goal_pose.pose.position.x,
                resolution.goal_pose.pose.position.y,
            ],
        },
        "module3": {
            "verified": verification.verified,
            "error_code": verification.error_code,
            "checks": check_rows(verification),
            "planning_length": verification.planning_length,
            "planning_time": verification.planning_time,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = LiveObserver()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        readiness = wait_for(
            executor,
            lambda: node.readiness if node.readiness and node.readiness.ready else None,
            args.timeout,
            "Module3 verification readiness",
        )
        wait_for(
            executor,
            lambda: (
                node_subscribes(node, "/user_instruction", "llm_agent_node")
                and node_subscribes(
                    node, "/semantic_navigation/intent", "semantic_graph_node"
                )
                and node_subscribes(
                    node,
                    "/semantic_navigation/resolution",
                    "verification_node",
                )
            ),
            args.timeout,
            "Module1-3 topic subscriptions",
        )
        node_names = sorted(name for name, _namespace in node.get_node_names_and_namespaces())
        forbidden = {"navigation_executor_node", "semantic_navigation_compatibility"}
        assert forbidden.isdisjoint(node_names)

        report = {
            "mode": "live_cloud_isaac_nav2",
            "readiness": {
                "ready": readiness.ready,
                "message": readiness.message,
                "checks": check_rows(readiness),
            },
            "nodes": node_names,
            "cases": [],
        }
        for case in CASES:
            result = run_case(node, executor, case, args.timeout)
            report["cases"].append(result)
            print(json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
        report["summary"] = f"{len(report['cases'])}/{len(CASES)} PASS"
        rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
        if args.output:
            args.output.write_text(rendered + "\n", encoding="utf-8")
        print(report["summary"], flush=True)
        return 0
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
