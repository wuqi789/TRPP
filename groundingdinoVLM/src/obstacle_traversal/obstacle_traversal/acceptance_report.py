#!/usr/bin/env python3
"""Evaluate real three-modal Isaac evidence and hash the corresponding rosbag."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

import yaml


COMMON_REQUIRED_TOPICS = {
    "/user_instruction", "/semantic_navigation/intent",
    "/semantic_navigation/resolution", "/semantic_navigation/verification",
    "/semantic_navigation/status", "/neupan_initial_path", "/scan_raw", "/scan",
    "/scan_removed", "/obstacle_traversal/status",
    "/obstacle_traversal/filter_state", "/neupan_cmd_vel_raw",
    "/obstacle_traversal/approach_cmd_vel",
    "/obstacle_traversal/mechanical_assessment",
    "/piper/yolo_obstacle_result", "/piper/yolo_ready",
    "/neupan_cmd_vel", "/cmd_vel", "/tf", "/odom",
    "/isaac_joint_states", "/isaac_joint_command",
    "/semantic_mapping/obstacle_cloud", "/semantic_mapping/labels",
    "/robot_marker",
}

SCENARIOS = {"curtain", "movable_box", "fixed_box"}
APPROACHING = 7
ALIGNING = 8
ARM_READY = 9
PROBING = 10
ARM_RETURNING = 11
FUSING = 12
RECOVERING = 13
AUTHORIZED = 3
TRAVERSING = 4
REJECTED = 5


def required_topics_for_scenario(
    scenario: str, *, core_only: bool = False
) -> set[str]:
    topics = set(COMMON_REQUIRED_TOPICS)
    if core_only:
        topics.difference_update({
            "/user_instruction",
            "/semantic_navigation/intent",
            "/semantic_navigation/resolution",
            "/semantic_navigation/verification",
        })
    if scenario in {"movable_box", "fixed_box"}:
        topics.add("/isaac/acceptance_furniture_pose")
    return topics


def load_events(path: Path) -> list[dict]:
    events = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict) or "event" not in value:
            raise ValueError(f"invalid evidence line {number}")
        events.append(value)
    return events


def _distance(first, last) -> float:
    return math.dist((float(first["x"]), float(first["y"])), (float(last["x"]), float(last["y"])))


def evaluate(
    scenario: str, events: list[dict], *, core_only: bool = False
) -> tuple[list[dict], dict]:
    if scenario not in SCENARIOS:
        raise ValueError("scenario must be curtain, movable_box, or fixed_box")
    by_type = {}
    for event in events:
        by_type.setdefault(event["event"], []).append(event)
    checks = []

    def check(name: str, passed: bool, evidence) -> None:
        checks.append({"name": name, "passed": bool(passed), "evidence": evidence})

    common_requests = set()
    if core_only:
        check("acceptance_mode", True, "core_only_real_providers")
    else:
        check(
            "instruction_entity", any(x.get("contains_expected_entity") for x in by_type.get("instruction", [])),
            "bathroom_basin_0001" if scenario == "curtain" else "living_room_cabinet_0008",
        )
        stages = ["module1", "module2", "module3", "module4"]
        request_sets = [
            {str(x.get("request_id", "")) for x in by_type.get(stage, []) if x.get("request_id")}
            for stage in stages
        ]
        common_requests = set.intersection(*request_sets) if all(request_sets) else set()
        check("module1_to_module4_request_id", bool(common_requests), sorted(common_requests))
        check("module2_resolved", any(x.get("resolved") for x in by_type.get("module2", [])), None)
        check("module3_verified", any(x.get("verified") for x in by_type.get("module3", [])), None)
    traversal = by_type.get("traversal", [])
    triggered = [x for x in traversal if int(x.get("state", -1)) == APPROACHING]
    terminal_decisions = [
        x for x in traversal
        if int(x.get("state", -1)) in {AUTHORIZED, TRAVERSING, REJECTED, 6}
    ]
    check("path_obstacle_triggered", bool(triggered), len(triggered))
    offsets = [
        abs(float(x.get("dino_started_offset_ms", 0.0)) - float(x.get("llmdecision_started_offset_ms", 0.0)))
        for x in triggered
    ]
    check("parallel_start_within_200ms", bool(offsets) and max(offsets) <= 200.0, max(offsets, default=None))
    graph_audits = by_type.get("graph_audit", [])
    mock_nodes = sorted({node for event in graph_audits for node in event.get("mock_nodes", [])})
    check("no_mock_nodes", bool(graph_audits) and not mock_nodes, mock_nodes)
    if core_only:
        graph_nodes = {
            node for event in graph_audits for node in event.get("nodes", [])
        }
        check(
            "core_acceptance_driver_present",
            "/core_acceptance_driver" in graph_nodes,
            sorted(graph_nodes),
        )
    action_services = graph_audits[-1].get("action_services", {}) if graph_audits else {}
    check(
        "all_real_action_services_available",
        bool(action_services) and all(bool(value) for value in action_services.values()),
        action_services,
    )
    piper_publishers = sorted({
        node for event in graph_audits
        for node in event.get("piper_command_publishers", [])
    })
    piper_subscribers = sorted({
        node for event in graph_audits
        for node in event.get("piper_command_subscribers", [])
    })
    check(
        "unique_piper_command_publisher",
        len(piper_publishers) == 1 and bool(piper_subscribers),
        {"publishers": piper_publishers, "subscribers": piper_subscribers},
    )
    yolo_readiness = [
        value for value in by_type.get("readiness", [])
        if value.get("name") == "piper_yolo_ready"
    ]
    check(
        "piper_yolo_ready",
        any(bool(value.get("ready")) for value in yolo_readiness),
        yolo_readiness[-1] if yolo_readiness else None,
    )
    navigation_success = any(
        x.get("terminal") and x.get("state") == "SUCCEEDED"
        for x in by_type.get("module4", [])
    )
    check("navigation_succeeded", navigation_success, None)

    final = max(terminal_decisions, key=lambda x: x.get("wall_time", 0.0), default={})
    probabilities = {
        "dino": float(final.get("dino_probability", 0.0)),
        "llm": float(final.get("pushability_probability", 0.0)),
        "arm": float(final.get("mechanical_probability", 0.0)),
        "fusion": float(final.get("fusion_probability", 0.0)),
    }
    check(
        "real_visual_results_recorded",
        bool(final)
        and probabilities["llm"] > 0.0
        and (scenario == "fixed_box" or probabilities["dino"] > 0.0),
        probabilities,
    )

    approach_feedback = by_type.get("approach_feedback", [])
    working_clearances = [
        float(value.get("target_clearance_m")) for value in approach_feedback
        if int(value.get("phase", -1)) == 1
        and math.isfinite(float(value.get("target_clearance_m", math.nan)))
    ]
    check(
        "stopped_at_0_20m",
        bool(working_clearances)
        and any(0.18 <= value <= 0.22 for value in working_clearances),
        working_clearances[-10:],
    )
    external_clearances = [
        float(value.get("external_clearance_m")) for value in approach_feedback
        if math.isfinite(float(value.get("external_clearance_m", math.nan)))
    ]
    check(
        "external_clearance_at_least_0_10m",
        bool(external_clearances) and min(external_clearances) >= 0.10,
        min(external_clearances, default=None),
    )

    assessments = by_type.get("mechanical_assessment", [])
    assessment = assessments[-1] if assessments else {}
    expected_state = {"curtain": 0, "movable_box": 1, "fixed_box": 2}[scenario]
    expected_yolo = {2} if scenario == "curtain" else {0, 1, 2}
    check(
        "expected_piper_yolo_classification",
        bool(assessment)
        and bool(assessment.get("vision_backend_ok"))
        and int(assessment.get("vision_classification", -1)) in expected_yolo
        and int(assessment.get("vision_votes", 0)) >= 3,
        {
            "classification": assessment.get("vision_classification"),
            "confidence": assessment.get("vision_confidence"),
            "backend_ok": assessment.get("vision_backend_ok"),
            "votes": assessment.get("vision_votes"),
            "samples": assessment.get("vision_samples"),
            "expected": sorted(expected_yolo),
        },
    )
    check(
        "expected_mechanical_classification",
        bool(assessment) and int(assessment.get("state", -1)) == expected_state,
        assessment,
    )
    check(
        "piper_return_confirmed",
        bool(assessment) and bool(assessment.get("arm_returned")),
        assessment.get("arm_returned"),
    )
    if scenario == "movable_box":
        check(
            "movable_box_displacement_at_least_0_02m",
            float(assessment.get("object_displacement_m", 0.0)) >= 0.02,
            assessment.get("object_displacement_m"),
        )

    assessment_request_id = str(assessment.get("request_id", ""))
    arm_windows = [
        value for value in traversal
        if str(value.get("request_id", "")) == assessment_request_id
        if int(value.get("state", -1)) in {ARM_READY, PROBING, ARM_RETURNING, FUSING}
    ]
    arm_start = min((float(x["wall_time"]) for x in arm_windows), default=None)
    arm_end = float(assessment["wall_time"]) if assessment else None
    arm_commands = [
        value for value in by_type.get("cmd_executed", [])
        if str(value.get("request_id", "")) == assessment_request_id
        if arm_start is not None and arm_end is not None
        and arm_start <= float(value["wall_time"]) <= arm_end
    ]
    arm_maximum = max(
        (
            max(abs(float(value.get("linear_x", 0.0))),
                abs(float(value.get("angular_z", 0.0))))
            for value in arm_commands
        ),
        default=math.inf,
    )
    check(
        "base_zero_during_piper",
        bool(arm_commands) and arm_maximum <= 0.001,
        arm_maximum,
    )

    trigger_time = min((float(x["wall_time"]) for x in triggered), default=None)
    request_ids = {str(x.get("request_id", "")) for x in triggered}
    approach_commands = [
        value for value in by_type.get("approach_cmd", [])
        if str(value.get("request_id", "")) in request_ids
    ]
    approach_executed = [
        value for value in by_type.get("cmd_executed", [])
        if str(value.get("request_id", "")) in request_ids
        if trigger_time is not None and arm_start is not None
        and trigger_time <= float(value["wall_time"]) < arm_start
        and max(
            abs(float(value.get("linear_x", 0.0))),
            abs(float(value.get("angular_z", 0.0))),
        ) > 0.001
    ]
    ownership_matches = []
    for executed in approach_executed:
        nearest = min(
            approach_commands,
            key=lambda value: abs(
                float(value["wall_time"]) - float(executed["wall_time"])
            ),
            default=None,
        )
        ownership_matches.append(bool(
            nearest is not None
            and abs(float(nearest["wall_time"]) - float(executed["wall_time"])) <= 0.10
            and abs(float(nearest.get("linear_x", 0.0)) - float(executed.get("linear_x", 0.0))) <= 0.02
            and abs(float(nearest.get("angular_z", 0.0)) - float(executed.get("angular_z", 0.0))) <= 0.02
        ))
    check(
        "no_neupan_command_ownership_before_fusion",
        bool(approach_commands) and bool(approach_executed)
        and all(ownership_matches),
        {"approach_commands": len(approach_commands),
         "executed_nonzero": len(approach_executed),
         "matches": ownership_matches},
    )

    raw_by_stamp = {
        value.get("stamp"): value.get("ranges_sha256")
        for value in by_type.get("scan_raw", []) if value.get("stamp")
    }
    filtered_before = [
        value for value in by_type.get("scan_filtered", [])
        if not any(
            event.get("active") and float(event.get("wall_time", 0.0))
            <= float(value.get("wall_time", 0.0))
            for event in by_type.get("filter", [])
        )
    ]
    preauth_pairs = [
        (raw_by_stamp[value.get("stamp")], value.get("ranges_sha256"))
        for value in filtered_before if value.get("stamp") in raw_by_stamp
    ]
    check(
        "full_scan_preserved_before_authorization",
        bool(preauth_pairs) and all(a == b for a, b in preauth_pairs),
        len(preauth_pairs),
    )

    filters = by_type.get("filter", [])
    maximum_removed = max((int(x.get("removed_points", 0)) for x in filters), default=0)
    if scenario in {"curtain", "movable_box"}:
        check("fusion_authorized", probabilities["fusion"] >= 0.75, probabilities)
        check("authorized_scan_removal_nonzero", maximum_removed > 0, maximum_removed)
        check(
            "authorized_or_traversing_state_observed",
            any(int(x.get("state", -1)) in {AUTHORIZED, TRAVERSING} for x in traversal),
            None,
        )
    else:
        check("fixed_box_scan_never_removed", maximum_removed == 0, maximum_removed)
        check(
            "fixed_box_rejected",
            any(int(x.get("state", -1)) == REJECTED for x in traversal),
            None,
        )
        recovering = [x for x in traversal if int(x.get("state", -1)) == RECOVERING]
        rejected = [x for x in traversal if int(x.get("state", -1)) == REJECTED]
        recovery_distance = None
        if recovering and rejected and all(
            value.get(name) is not None
            for value in (recovering[0], rejected[-1])
            for name in ("robot_x", "robot_y")
        ):
            recovery_distance = math.dist(
                (float(recovering[0]["robot_x"]), float(recovering[0]["robot_y"])),
                (float(rejected[0]["robot_x"]), float(rejected[0]["robot_y"])),
            )
        check(
            "controlled_recovery_0_30m",
            recovery_distance is not None and 0.27 <= recovery_distance <= 0.33,
            recovery_distance,
        )
    authorization_windows = {}
    for event in filters:
        if event.get("active") and event.get("request_id"):
            authorization_windows.setdefault(str(event["request_id"]), []).append(
                float(event["wall_time"])
            )
    if authorization_windows:
        durations = []
        speeds = []
        for times in authorization_windows.values():
            start, end = min(times), max(times)
            durations.append(end - start)
            speeds.extend(
                abs(float(event.get("linear_x", 0.0)))
                for event in by_type.get("cmd_gated", [])
                if start <= float(event["wall_time"]) <= end
            )
        check(
            "authorized_speed_at_most_0_15mps",
            bool(speeds) and max(speeds) <= 0.150001,
            max(speeds, default=None),
        )
        check(
            "filter_duration_at_most_30s",
            bool(durations) and max(durations) <= 30.25,
            max(durations, default=None),
        )
    elif scenario in {"curtain", "movable_box"}:
        check("authorized_speed_at_most_0_15mps", False, None)
        check("filter_duration_at_most_30s", False, None)
    summary = {
        "scenario": scenario, "request_ids": sorted(common_requests),
        "maximum_removed_points": maximum_removed,
        "probabilities": probabilities,
        "mechanical_state": assessment.get("state"),
        "mechanical_displacement_m": assessment.get("object_displacement_m"),
    }
    return checks, summary


def bag_manifest(bag: Path) -> tuple[dict, set[str]]:
    metadata = bag / "metadata.yaml"
    if not metadata.is_file():
        raise FileNotFoundError(f"rosbag metadata is missing: {metadata}")
    document = yaml.safe_load(metadata.read_text(encoding="utf-8")) or {}
    topics = set()

    def walk(value):
        if isinstance(value, dict):
            metadata_value = value.get("topic_metadata")
            if isinstance(metadata_value, dict) and metadata_value.get("name"):
                topics.add(str(metadata_value["name"]))
            for nested in value.values():
                walk(nested)
        elif isinstance(value, list):
            for nested in value:
                walk(nested)

    walk(document)
    files = []
    aggregate = hashlib.sha256()
    for path in sorted(value for value in bag.rglob("*") if value.is_file()):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        relative = str(path.relative_to(bag))
        files.append({"path": relative, "sha256": digest, "bytes": path.stat().st_size})
        aggregate.update(relative.encode() + b"\0" + bytes.fromhex(digest))
    return {"sha256": aggregate.hexdigest(), "files": files}, topics


def build_report(
    scenario: str, events_path: Path, bag: Path, *, core_only: bool = False
) -> dict:
    events = load_events(events_path)
    checks, summary = evaluate(scenario, events, core_only=core_only)
    manifest, topics = bag_manifest(bag)
    missing = sorted(
        required_topics_for_scenario(scenario, core_only=core_only) - topics
    )
    checks.append({"name": "required_rosbag_topics", "passed": not missing, "evidence": missing})
    return {
        "schema_version": 2, "real_only": True, "core_only": core_only,
        "scenario": scenario,
        "passed": all(value["passed"] for value in checks),
        "checks": checks, "summary": summary,
        "events_sha256": hashlib.sha256(events_path.read_bytes()).hexdigest(),
        "rosbag": {**manifest, "topics": sorted(topics)},
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=tuple(sorted(SCENARIOS)), required=True)
    parser.add_argument("--events", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--core-only", action="store_true")
    args = parser.parse_args(argv)
    report = build_report(
        args.scenario, args.events, args.bag, core_only=args.core_only
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"[{'PASS' if report['passed'] else 'FAIL'}] {args.scenario} real acceptance: {args.output}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
