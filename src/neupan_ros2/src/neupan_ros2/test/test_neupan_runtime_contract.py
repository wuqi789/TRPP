from pathlib import Path

import numpy as np

from neupan_ros2.neupan_node import points_inside_padded_footprint


SOURCE = (
    Path(__file__).parents[1] / "neupan_ros2" / "neupan_node.py"
).read_text(encoding="utf-8")


def test_subscriptions_are_retained_for_node_lifetime():
    for name in ("scan_sub", "plan_sub", "clicked_point_sub", "scout_status_sub"):
        assert f"self.{name} = self.create_subscription(" in SOURCE


def test_arrival_without_optional_visualization_paths_does_not_crash():
    assert 'info["opt_state_list"]' not in SOURCE
    assert 'info["ref_state_list"]' not in SOURCE
    assert 'info.get("opt_state_list")' in SOURCE
    assert 'info.get("ref_state_list")' in SOURCE


def test_initial_path_is_latched_for_traversal_and_late_joiners():
    assert "QoSDurabilityPolicy.TRANSIENT_LOCAL" in SOURCE
    assert "initial_path_qos" in SOURCE


def test_isaac_padding_removes_recorded_self_return_but_keeps_obstacles():
    half_length = 0.615 / 2.0
    half_width = 0.585 / 2.0
    matrix = np.array([
        [1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0],
    ])
    bounds = np.array([[half_length], [half_length], [half_width], [half_width]])
    points = np.array([
        [0.281023, 0.281023, 0.60],
        [-0.320365, -0.350000, 0.00],
    ])

    exact = points_inside_padded_footprint(points, matrix, bounds, 0.0)
    padded = points_inside_padded_footprint(points, matrix, bounds, 0.04)

    assert exact.tolist() == [False, False, False]
    assert padded.tolist() == [True, False, False]


def test_footprint_padding_is_invariant_to_halfspace_row_scale():
    matrix = np.array([[2.0, 0.0], [0.0, -3.0]])
    bounds = np.array([[0.615], [0.8775]])
    points = np.array([[0.33], [-0.32]])

    assert points_inside_padded_footprint(
        points, matrix, bounds, 0.04
    ).tolist() == [True]
