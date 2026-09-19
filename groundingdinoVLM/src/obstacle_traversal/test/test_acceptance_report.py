from obstacle_traversal.acceptance_report import (
    evaluate,
    required_topics_for_scenario,
)


def common_events():
    events = [{"event": "instruction", "contains_expected_entity": True}]
    for stage in ("module1", "module2", "module3", "module4"):
        event = {"event": stage, "request_id": "nav-1"}
        if stage == "module2":
            event["resolved"] = True
        if stage == "module3":
            event["verified"] = True
        if stage == "module4":
            event.update(state="SUCCEEDED", terminal=True)
        events.append(event)
    events.extend([
        {
            "event": "graph_audit",
            "nodes": ["/core_acceptance_driver", "/piper_probe_server"],
            "mock_nodes": [],
            "action_services": {
                "dino": True, "llmdecision": True,
                "approach": True, "piper": True,
            },
            "piper_command_publishers": ["/piper_probe_server"],
            "piper_command_subscribers": ["/isaac_sim"],
        },
        {
            "event": "readiness", "name": "piper_yolo_ready", "ready": True,
        },
        {
            "event": "traversal", "wall_time": 1.0, "state": 7,
            "request_id": "traversal-1", "dino_started_offset_ms": 2.0,
            "llmdecision_started_offset_ms": 5.0,
            "robot_x": 0.0, "robot_y": 0.0,
            "obstacle_x": 1.0, "obstacle_y": 0.0,
        },
        {
            "event": "approach_feedback", "wall_time": 1.8,
            "request_id": "traversal-1", "phase": 1,
            "target_clearance_m": 0.20, "external_clearance_m": 0.15,
        },
        {
            "event": "traversal", "wall_time": 2.0, "state": 9,
            "request_id": "traversal-1", "approach_clearance_m": 0.20,
            "robot_x": 0.49, "robot_y": 0.0,
        },
        {
            "event": "approach_cmd", "wall_time": 1.50,
            "request_id": "traversal-1", "linear_x": 0.08, "angular_z": 0.0,
        },
        {
            "event": "cmd_executed", "wall_time": 1.51,
            "request_id": "traversal-1", "linear_x": 0.08,
            "angular_z": 0.0,
        },
        {
            "event": "traversal", "wall_time": 2.1, "state": 10,
            "request_id": "traversal-1", "robot_x": 0.49, "robot_y": 0.0,
        },
        {
            "event": "cmd_executed", "wall_time": 2.5,
            "request_id": "traversal-1", "linear_x": 0.0,
            "angular_z": 0.0,
        },
        {"event": "scan_raw", "wall_time": 1.2, "stamp": "1.200", "ranges_sha256": "same"},
        {"event": "scan_filtered", "wall_time": 1.21, "stamp": "1.200", "ranges_sha256": "same"},
    ])
    return events


def add_mechanical(
    events,
    state,
    displacement,
    probability=0.9,
    yolo=None,
    yolo_backend_ok=True,
):
    vision_classification = (2 if state == 0 else 0) if yolo is None else int(yolo)
    events.extend([
        {
            "event": "mechanical_assessment", "wall_time": 4.0,
            "request_id": "traversal-1", "state": state,
            "mechanical_probability": probability,
            "object_displacement_m": displacement, "arm_returned": True,
            "vision_classification": vision_classification,
            "vision_confidence": 0.85 if vision_classification == 2 else 0.0,
            "vision_backend_ok": bool(yolo_backend_ok),
            "vision_votes": 3,
            "vision_samples": 5,
            "error_code": "",
        },
        {
            "event": "traversal", "wall_time": 4.1, "state": 12,
            "request_id": "traversal-1", "robot_x": 0.49, "robot_y": 0.0,
        },
    ])


def add_authorized(events):
    events.extend([
        {
            "event": "traversal", "wall_time": 5.0, "state": 3,
            "request_id": "traversal-1", "dino_probability": 0.9,
            "pushability_probability": 0.9, "mechanical_probability": 0.9,
            "fusion_probability": 0.9, "arm_returned": True,
        },
        {
            "event": "filter", "wall_time": 5.0, "request_id": "traversal-1",
            "active": True, "removed_points": 8,
        },
        {
            "event": "filter", "wall_time": 6.0, "request_id": "traversal-1",
            "active": True, "removed_points": 6,
        },
        {"event": "cmd_gated", "wall_time": 5.5, "linear_x": 0.15},
    ])


def test_curtain_real_three_modal_report_passes_complete_contract():
    events = common_events()
    add_mechanical(events, state=0, displacement=0.0)
    add_authorized(events)
    checks, summary = evaluate("curtain", events)
    assert all(value["passed"] for value in checks), checks
    assert summary["mechanical_state"] == 0


def test_only_rigid_acceptance_scenarios_require_furniture_pose_topic():
    pose_topic = "/isaac/acceptance_furniture_pose"
    assert pose_topic not in required_topics_for_scenario("curtain")
    assert pose_topic in required_topics_for_scenario("movable_box")
    assert pose_topic in required_topics_for_scenario("fixed_box")


def test_core_acceptance_is_explicit_and_does_not_require_module_topics():
    events = common_events()
    add_mechanical(events, state=0, displacement=0.0)
    add_authorized(events)
    checks, _ = evaluate("curtain", events, core_only=True)
    assert all(value["passed"] for value in checks), checks
    required = required_topics_for_scenario("curtain", core_only=True)
    assert "/semantic_navigation/status" in required
    assert "/semantic_navigation/intent" not in required
    assert "/semantic_navigation/resolution" not in required
    assert "/semantic_navigation/verification" not in required


def test_piper_zero_velocity_check_ignores_an_earlier_request():
    events = common_events()
    events.append({
        "event": "cmd_executed", "wall_time": 2.6,
        "request_id": "traversal-earlier", "linear_x": 0.4,
        "angular_z": 0.2,
    })
    add_mechanical(events, state=0, displacement=0.0)
    add_authorized(events)
    checks, _ = evaluate("curtain", events)
    stopped = next(
        value for value in checks if value["name"] == "base_zero_during_piper"
    )
    assert stopped["passed"], stopped


def test_movable_box_requires_verified_two_centimeter_displacement():
    events = common_events()
    add_mechanical(events, state=1, displacement=0.019)
    add_authorized(events)
    checks, _ = evaluate("movable_box", events)
    displacement = next(
        value for value in checks
        if value["name"] == "movable_box_displacement_at_least_0_02m"
    )
    assert not displacement["passed"]


def test_fixed_box_requires_anchored_no_filter_and_0_30m_recovery():
    events = common_events()
    add_mechanical(events, state=2, displacement=0.0, probability=0.0)
    events.extend([
        {
            "event": "traversal", "wall_time": 5.0, "state": 13,
            "request_id": "traversal-1", "robot_x": 0.49, "robot_y": 0.0,
        },
        {
            "event": "traversal", "wall_time": 8.0, "state": 5,
            "request_id": "traversal-1", "robot_x": 0.19, "robot_y": 0.0,
            "dino_probability": 0.9, "pushability_probability": 0.2,
            "mechanical_probability": 0.0, "fusion_probability": 0.275,
            "arm_returned": True,
        },
        {"event": "filter", "wall_time": 8.0, "active": False, "removed_points": 0},
    ])
    checks, _ = evaluate("fixed_box", events)
    assert all(value["passed"] for value in checks), checks


def test_mock_node_or_second_piper_publisher_fails_real_acceptance():
    events = common_events()
    audit = next(value for value in events if value["event"] == "graph_audit")
    audit["mock_nodes"] = ["/mock_provider"]
    audit["piper_command_publishers"].append("/legacy_adapter")
    add_mechanical(events, state=0, displacement=0.0)
    add_authorized(events)
    checks, _ = evaluate("curtain", events)
    assert not next(value for value in checks if value["name"] == "no_mock_nodes")["passed"]
    assert not next(
        value for value in checks
        if value["name"] == "unique_piper_command_publisher"
    )["passed"]


def test_scan_must_match_raw_before_authorization():
    events = common_events()
    filtered = next(value for value in events if value["event"] == "scan_filtered")
    filtered["ranges_sha256"] = "changed"
    add_mechanical(events, state=0, displacement=0.0)
    add_authorized(events)
    checks, _ = evaluate("curtain", events)
    check = next(
        value for value in checks
        if value["name"] == "full_scan_preserved_before_authorization"
    )
    assert not check["passed"]


def test_wrong_mechanical_classification_cannot_pass_scenario_report():
    events = common_events()
    add_mechanical(events, state=2, displacement=0.0, probability=0.0)
    add_authorized(events)
    checks, _ = evaluate("curtain", events)
    classification = next(
        value for value in checks
        if value["name"] == "expected_mechanical_classification"
    )
    assert not classification["passed"]


def test_wrong_yolo_classification_cannot_pass_curtain_report():
    events = common_events()
    add_mechanical(events, state=0, displacement=0.0, yolo=1)
    add_authorized(events)
    checks, _ = evaluate("curtain", events)
    classification = next(
        value for value in checks
        if value["name"] == "expected_piper_yolo_classification"
    )
    assert not classification["passed"]


def test_yolo_backend_failure_cannot_pass_curtain_report():
    events = common_events()
    add_mechanical(
        events, state=0, displacement=0.0, yolo=2, yolo_backend_ok=False
    )
    add_authorized(events)
    checks, _ = evaluate("curtain", events)
    classification = next(
        value for value in checks
        if value["name"] == "expected_piper_yolo_classification"
    )
    assert not classification["passed"]
