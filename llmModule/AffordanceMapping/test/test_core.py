import json
import math

import numpy as np
import pytest

from llm_semantic_affordance.core import (
    Detection2D,
    InstanceMap,
    KeyframeScheduler,
    LocalizedObject,
    PushabilityResponseError,
    localize_detection,
    parse_pushability_response,
)


def response(**overrides):
    value = {
        "label": "chair",
        "bbox_px": [1, 1, 8, 8],
        "pushable": True,
        "confidence": 0.91,
        "reason": "freestanding",
    }
    value.update(overrides)
    return json.dumps({"objects": [value]})


def test_strict_pushability_response_contract():
    detection = parse_pushability_response(response(), 10, 10)[0]
    assert detection == Detection2D(
        "chair", (1, 1, 8, 8), True, 0.91, "freestanding"
    )
    with pytest.raises(PushabilityResponseError, match="Markdown"):
        parse_pushability_response(f"```json\n{response()}\n```", 10, 10)
    with pytest.raises(PushabilityResponseError, match="schema"):
        parse_pushability_response(response(extra=True), 10, 10)
    with pytest.raises(PushabilityResponseError, match="out of bounds"):
        parse_pushability_response(response(bbox_px=[1, 1, 10, 8]), 10, 10)
    with pytest.raises(PushabilityResponseError, match="confidence"):
        parse_pushability_response(response(confidence=float("nan")), 10, 10)


def test_overlapping_duplicate_boxes_keep_highest_confidence():
    raw = json.dumps({"objects": [
        json.loads(response(confidence=0.80))["objects"][0],
        json.loads(response(confidence=0.95))["objects"][0],
    ]})
    result = parse_pushability_response(raw, 10, 10)
    assert len(result) == 1
    assert result[0].confidence == 0.95


def test_sparse_keyframe_scheduler_requires_time_and_motion():
    scheduler = KeyframeScheduler(0.5, math.radians(30.0), 5.0)
    assert scheduler.should_enqueue(0.0, 0.0, 0.0, 0.0)
    scheduler.mark_enqueued(0.0, 0.0, 0.0, 0.0)
    assert not scheduler.should_enqueue(1.0, 0.0, 0.0, 4.9)
    assert scheduler.should_enqueue(0.5, 0.0, 0.0, 5.0)
    assert scheduler.should_enqueue(0.0, 0.0, math.radians(30.0), 5.0)
    assert not scheduler.should_enqueue(0.49, 0.0, math.radians(29.0), 5.0)


def test_rgbd_detection_localizes_in_map_frame():
    depth = np.ones((10, 10), dtype=np.float32)
    class_ids = np.full((10, 10), 19, dtype=np.uint8)
    confidence = np.ones((10, 10), dtype=np.float32)
    self_mask = np.zeros((10, 10), dtype=bool)
    transform = np.eye(4)
    transform[:3, 3] = (2.0, 3.0, 0.5)
    value = localize_detection(
        Detection2D("chair", (1, 1, 8, 8), True, 0.9, "visible"),
        depth,
        class_ids,
        confidence,
        self_mask,
        (10.0, 10.0, 5.0, 5.0),
        transform,
        123,
        minimum_points=10,
    )
    assert value.semantic_id == 19
    assert value.center[0] == pytest.approx(1.95, abs=0.1)
    assert value.center[1] == pytest.approx(2.95, abs=0.1)
    assert value.center[2] == pytest.approx(1.5)


def localized(x, pushable, confidence, stamp=1):
    return LocalizedObject(
        "chair", "19", 19,
        (x, 0.0, 0.5), (0.4, 0.4, 1.0),
        (x - 0.2, -0.2, x + 0.2, 0.2),
        pushable, confidence, "evidence", stamp,
    )


def test_instance_association_and_highest_confidence_conflict_rule():
    instances = InstanceMap(iou_threshold=0.20, distance_threshold=0.60)
    first = instances.update(localized(0.0, True, 0.90))
    second = instances.update(localized(0.2, False, 0.95, 2))
    assert first.instance_id == second.instance_id
    assert not second.pushable
    assert second.confidence == 0.95
    third = instances.update(localized(0.1, True, 0.99, 3))
    assert third.pushable
    assert third.confidence == 0.99
    assert third.observation_count == 3
    other = instances.update(localized(2.0, True, 0.90, 4))
    assert other.instance_id != first.instance_id
    assert len(instances) == 2


def test_equal_positive_and_negative_confidence_fails_closed():
    instances = InstanceMap()
    instances.update(localized(0.0, True, 0.90))
    result = instances.update(localized(0.0, False, 0.90, 2))
    assert not result.pushable
    assert result.confidence == 0.90
