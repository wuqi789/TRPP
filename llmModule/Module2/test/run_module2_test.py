#!/usr/bin/env python3
"""Run an end-to-end, ROS-independent test of all Module2 core functions."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import yaml


MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from builder import build_graph, load_map
from query import ground_entity, query_entities, query_relation, query_topology, relative_pose


DEFAULT_MAP = MODULE_ROOT / "maps" / "DCT_hospital_demo.yaml"
DEFAULT_INPUT = Path(__file__).resolve().parent / "mock_module1_output.yaml"


class PracticeReport:
    def __init__(self) -> None:
        self.passed = 0
        self.failed = 0

    def check(self, name: str, condition: bool, output: Any) -> None:
        status = "PASS" if condition else "FAIL"
        print(f"\n[{status}] {name}")
        print(json.dumps(output, ensure_ascii=False, indent=2))
        if condition:
            self.passed += 1
        else:
            self.failed += 1

    def summary(self) -> int:
        print("\n" + "=" * 60)
        print(f"Module2 测试完成：{self.passed} passed, {self.failed} failed")
        return 0 if self.failed == 0 else 1


def load_mock_intent(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError("模拟 Module1 输出必须是 YAML mapping")
    required = {"goal_object", "reference_object", "relation", "strategy", "constraints"}
    missing = sorted(required.difference(data))
    if missing:
        raise ValueError(f"模拟 Module1 输出缺少字段：{missing}")
    return data


def node_output(node) -> dict[str, Any]:
    return {
        "id": node.id,
        "type": node.type,
        "name": node.name,
        "position": node.position.to_dict(),
        "properties": dict(node.properties),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test all Module2 semantic graph core functions")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="mock Module1 output YAML")
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP, help="semantic map YAML")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = PracticeReport()

    print("=" * 60)
    print("Module2 Semantic Graph 独立功能测试")
    print(f"地图：{args.map}")
    print(f"模拟 Module1 输入：{args.input}")

    try:
        intent = load_mock_intent(args.input)
        map_data = load_map(args.map)
        graph = build_graph(map_data)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"\n[FATAL] 初始化失败：{exc}", file=sys.stderr)
        return 2

    report.check("读取模拟 Module1 NavigationIntent", True, intent)

    graph_summary = {
        "node_count": graph.networkx_graph.number_of_nodes(),
        "edge_count": graph.networkx_graph.number_of_edges(),
        "node_ids": list(graph.networkx_graph.nodes),
    }
    report.check(
        "加载 DCT_hospital_demo.yaml 并构建 NetworkX SemanticGraph",
        graph_summary["node_count"] > 0 and graph_summary["edge_count"] > 0,
        graph_summary,
    )

    entities = query_entities(graph, str(intent["goal_object"]))
    entity_result = [node_output(node) for node in entities]
    report.check("查询目标实体", bool(entities), entity_result)

    relation_matches = query_relation(
        graph,
        str(intent["goal_object"]),
        str(intent["relation"]),
        str(intent["reference_object"]),
    )
    relation_result = {
        "source": intent["goal_object"],
        "relation": intent["relation"],
        "target": intent["reference_object"],
        "matched_ids": [node.id for node in relation_matches],
    }
    report.check("查询语义空间关系", bool(relation_matches), relation_result)

    candidate = ground_entity(
        graph,
        str(intent["goal_object"]),
        str(intent["reference_object"]),
        str(intent["relation"]),
    )
    pose_result = candidate.position.to_dict() if candidate else {}
    report.check(
        "获取目标坐标",
        candidate is not None and "x" in pose_result and "y" in pose_result,
        {"object_id": candidate.id if candidate else "", "pose": pose_result},
    )

    topology_path = query_topology(graph, "jackal_robot", candidate.id)
    report.check(
        "查询 Jackal 到消歧目标的拓扑路径",
        topology_path == ["jackal_robot", "hospital_scene", candidate.id],
        {"start": "jackal_robot", "goal": candidate.id, "path": topology_path},
    )

    origin = graph.get_node("jackal_robot")
    goal_x, goal_y, goal_theta = relative_pose(candidate.position, origin.position)
    semantic_goal = {
        "goal_id": candidate.id if candidate else "",
        "x": goal_x,
        "y": goal_y,
        "theta": goal_theta,
        "topology_context": [],
    }
    report.check(
        "生成 Module2 SemanticGoal 输出",
        candidate is not None and semantic_goal["goal_id"] != "",
        semantic_goal,
    )

    return report.summary()


if __name__ == "__main__":
    raise SystemExit(main())
