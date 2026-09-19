import pytest

from execution_core import RequestRegistry, StabilityTracker
from route_planning import GridPoint


def test_topic_and_action_request_ids_are_accepted_once():
    registry = RequestRegistry(4)
    assert registry.reserve_topic("request-1")
    assert not registry.reserve_topic("request-1")
    assert registry.accept_goal("request-1")
    assert registry.contains("request-1")
    assert not registry.accept_goal("request-1")
    assert not registry.reserve_topic("request-1")


def test_failed_topic_submission_can_be_released_and_retried():
    registry = RequestRegistry(4)
    assert registry.reserve_topic("request-1")
    registry.release_topic("request-1")
    assert registry.reserve_topic("request-1")


def test_request_history_is_bounded_and_evicts_oldest():
    registry = RequestRegistry(2)
    assert registry.accept_goal("request-1")
    assert registry.accept_goal("request-2")
    assert registry.accept_goal("request-3")
    assert not registry.contains("request-1")
    assert registry.contains("request-2")
    assert registry.contains("request-3")


def test_empty_request_id_is_never_accepted():
    registry = RequestRegistry()
    assert not registry.reserve_topic("")
    assert not registry.accept_goal("")


def test_stability_requires_continuous_dwell_and_resets_after_motion():
    tracker = StabilityTracker(
        movement_distance=0.05, hold_seconds=0.5, timeout=5.0
    )
    tracker.start(GridPoint(0.0, 0.0), 0.0)
    assert tracker.tick(GridPoint(0.04, 0.0), 0.49).kind == "WAITING"
    assert tracker.tick(GridPoint(0.06, 0.0), 0.50).kind == "WAITING"
    assert tracker.tick(GridPoint(0.06, 0.0), 0.99).kind == "WAITING"
    assert tracker.tick(GridPoint(0.06, 0.0), 1.0).kind == "STABLE"


def test_stability_reports_timeout_when_robot_never_settles():
    tracker = StabilityTracker(
        movement_distance=0.05, hold_seconds=0.5, timeout=1.0
    )
    tracker.start(GridPoint(0.0, 0.0), 0.0)
    assert tracker.tick(GridPoint(0.06, 0.0), 0.4).kind == "WAITING"
    assert tracker.tick(GridPoint(0.12, 0.0), 0.8).kind == "WAITING"
    assert tracker.tick(GridPoint(0.18, 0.0), 1.0).kind == "TIMEOUT"


def test_invalid_registry_capacity_is_rejected():
    with pytest.raises(ValueError):
        RequestRegistry(0)
