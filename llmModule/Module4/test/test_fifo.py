from execution_core import FifoTracker
from route_planning import GridPoint


def tracker(**kwargs):
    return FifoTracker(
        arrival_distance=kwargs.get("arrival_distance", 0.15),
        arrival_hold_seconds=kwargs.get("arrival_hold_seconds", 0.5),
        segment_timeout=kwargs.get("segment_timeout", 120.0),
        no_progress_timeout=kwargs.get("no_progress_timeout", 30.0),
        progress_distance=kwargs.get("progress_distance", 0.05),
        waypoint_pass_lateral_distance=kwargs.get(
            "waypoint_pass_lateral_distance", 0.75
        ),
        watchdog_enabled=kwargs.get("watchdog_enabled", True),
    )


def test_b_c_d_are_dispatched_in_strict_fifo_order_after_dwell():
    value = tracker()
    points = [GridPoint(1, 0), GridPoint(2, 0), GridPoint(3, 0)]
    assert value.start(points, GridPoint(0, 0), 0.0).target == points[0]
    assert value.tick(points[0], 1.0).kind == "NONE"
    assert value.tick(points[0], 1.49).kind == "NONE"
    assert value.tick(points[0], 1.5).target == points[1]
    assert value.tick(points[1], 2.0).kind == "NONE"
    assert value.tick(points[1], 2.5).target == points[2]
    assert value.tick(points[2], 3.0).kind == "NONE"
    assert value.tick(points[2], 3.5).kind == "COMPLETE"


def test_leaving_arrival_radius_resets_dwell():
    value = tracker()
    value.start([GridPoint(1, 0)], GridPoint(0, 0), 0.0)
    value.tick(GridPoint(1, 0), 1.0)
    value.tick(GridPoint(0.7, 0), 1.4)
    value.tick(GridPoint(1, 0), 1.5)
    assert value.tick(GridPoint(1, 0), 1.9).kind == "NONE"
    assert value.tick(GridPoint(1, 0), 2.0).kind == "COMPLETE"


def test_segment_timeout_fails():
    value = tracker(segment_timeout=2.0, no_progress_timeout=10.0)
    value.start([GridPoint(10, 0)], GridPoint(0, 0), 0.0)
    event = value.tick(GridPoint(0.1, 0), 2.0)
    assert (event.kind, event.code) == ("FAILED", "SEGMENT_TIMEOUT")


def test_no_progress_timeout_fails_and_motion_resets_window():
    value = tracker(segment_timeout=20.0, no_progress_timeout=2.0)
    value.start([GridPoint(10, 0)], GridPoint(0, 0), 0.0)
    assert value.tick(GridPoint(0.04, 0), 1.9).kind == "NONE"
    assert value.tick(GridPoint(0.06, 0), 2.0).kind == "NONE"
    assert value.tick(GridPoint(0.06, 0), 3.9).kind == "NONE"
    event = value.tick(GridPoint(0.06, 0), 4.0)
    assert (event.kind, event.code) == ("FAILED", "NO_PROGRESS")
    assert "robot=(0.060, 0.000)" in event.message
    assert "target=(10.000, 0.000)" in event.message
    assert "distance=9.940m" in event.message


def test_research_watchdog_valve_preserves_fifo_without_masking_arrival():
    value = tracker(
        segment_timeout=1.0, no_progress_timeout=1.0, watchdog_enabled=False
    )
    value.start([GridPoint(10, 0)], GridPoint(0, 0), 0.0)
    assert value.tick(GridPoint(0, 0), 100.0).kind == "NONE"
    value = tracker(watchdog_enabled=False)
    value.start([GridPoint(1, 0)], GridPoint(0, 0), 0.0)
    assert value.tick(GridPoint(1, 0), 100.0).kind == "NONE"
    assert value.tick(GridPoint(1, 0), 100.5).kind == "COMPLETE"


def test_empty_queue_completes_without_target():
    assert tracker().start([], GridPoint(0, 0), 0.0).kind == "COMPLETE"


def test_bounded_lateral_overshoot_advances_only_an_intermediate_waypoint():
    value = tracker(waypoint_pass_lateral_distance=0.75)
    points = [GridPoint(1, 0), GridPoint(2, 0)]
    value.start(points, GridPoint(0, 0), 0.0)
    event = value.tick(GridPoint(1.2, 0.5), 1.0)
    assert event.kind == "TARGET" and event.target == points[1]
    assert value.tick(GridPoint(2.2, 0.0), 2.0).kind == "NONE"


def test_large_lateral_detour_does_not_skip_direction_waypoint():
    value = tracker(waypoint_pass_lateral_distance=0.75)
    points = [GridPoint(1, 0), GridPoint(2, 0)]
    value.start(points, GridPoint(0, 0), 0.0)
    event = value.tick(GridPoint(1.2, 0.8), 1.0)
    assert event.kind == "NONE" and event.target == points[0]


def test_traversal_pause_shifts_segment_and_progress_deadlines():
    value = tracker(segment_timeout=5.0, no_progress_timeout=10.0)
    value.start([GridPoint(10, 0)], GridPoint(0, 0), 0.0)
    value.pause(2.0)
    assert value.tick(GridPoint(0, 0), 100.0).kind == "NONE"
    value.resume(102.0)
    assert value.segment_started == 100.0
    assert value.progress_at == 100.0
    assert value.tick(GridPoint(0, 0), 104.9).kind == "NONE"
    assert value.tick(GridPoint(0, 0), 105.0).code == "SEGMENT_TIMEOUT"
