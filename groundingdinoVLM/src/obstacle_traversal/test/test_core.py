import math

import numpy as np

from obstacle_traversal.core import (
    approach_path_command, closest_path_obstacle, expand_roi,
    fusion_probability, minimum_external_scan_clearance, movable_probability,
    nearest_tracked_point, outside_footprint_mask, project_roi,
    median_clearance, path_corridor_indices, point_clearance_from_footprint,
    path_tangent_heading_error, rear_sector_observable, scan_cluster_indices,
    filter_track_loss_in_completion_grace, rejected_cache_active, synchronized,
    target_band_observation, three_modal_authorized,
    traversal_path_command, traversal_sensor_inputs_ready, velocity_gate_mode,
)


def test_selected_obstacle_clearance_uses_body_edge():
    points = np.asarray([[0.51, 0.0, 0.0], [0.70, 0.1, 0.0]])
    assert math.isclose(point_clearance_from_footprint(points, 0.62, 0.586), 0.20)


def test_approach_command_stages_speed_and_stops_in_tolerance():
    path = [(0.0, 0.0), (2.0, 0.0)]
    far = approach_path_command((0.0, 0.0, 0.0), path, 1.0)
    near = approach_path_command((0.0, 0.0, 0.0), path, 0.30)
    reached = approach_path_command((0.0, 0.0, 0.0), path, 0.22)
    assert math.isclose(far.linear, 0.15)
    assert math.isclose(near.linear, 0.08)
    assert reached.reached and reached.linear == 0.0


def test_approach_command_turns_toward_locked_path():
    command = approach_path_command(
        (0.0, 0.1, 0.0), [(0.0, 0.0), (1.0, 0.0)], 0.8
    )
    assert command.angular < 0.0
    assert abs(command.cross_track_error) <= 0.11


def test_clearance_filter_uses_the_newest_three_finite_samples():
    assert math.isclose(median_clearance([0.30, 0.21, 0.19, 0.20], 3), 0.20)
    assert math.isclose(median_clearance([math.nan, 0.20], 3), 0.20)


def test_rejected_cache_remains_active_while_neupan_bypasses_obstacle():
    anchor = (0.75, 0.0)
    obstacle = (1.63, -0.075)
    assert rejected_cache_active(
        (1.71, 0.40), anchor, obstacle,
        motion_limit=1.0, obstacle_neighborhood=1.0,
    )
    assert not rejected_cache_active(
        (2.80, 0.0), anchor, obstacle,
        motion_limit=1.0, obstacle_neighborhood=1.0,
    )


def test_filter_track_loss_grace_preserves_the_full_pass_distance():
    assert not filter_track_loss_in_completion_grace(0.59, 0.80, 0.20)
    assert filter_track_loss_in_completion_grace(0.625, 0.80, 0.20)
    assert filter_track_loss_in_completion_grace(0.80, 0.80, 0.20)


def test_rear_positive_infinity_is_observed_free_space():
    ranges = [math.inf] * 360
    observable, coverage, samples = rear_sector_observable(
        ranges, -math.pi, 2.0 * math.pi / 360.0, 0.05, 30.0,
    )
    assert observable and math.isclose(coverage, 1.0)
    assert samples > 0


def test_rear_nan_sector_is_unobservable_but_finite_obstacle_is_observed():
    ranges = [math.inf] * 360
    for index in range(145, 215):
        ranges[index] = math.nan
    observable, coverage, _ = rear_sector_observable(
        ranges, 0.0, 2.0 * math.pi / 360.0, 0.05, 30.0,
    )
    assert not observable and coverage < 0.80
    ranges[180] = 0.35
    assert rear_sector_observable(
        ranges, 0.0, 2.0 * math.pi / 360.0, 0.05, 30.0,
        minimum_coverage=0.01,
    )[0]


def test_isaac_negative_one_no_return_requires_explicit_simulation_opt_in():
    ranges = [-1.0] * 720
    arguments = (-math.pi, 2.0 * math.pi / 720.0, 0.15, 8.0)
    assert not rear_sector_observable(ranges, *arguments)[0]
    observable, coverage, samples = rear_sector_observable(
        ranges, *arguments, negative_one_is_no_return=True
    )
    assert observable and math.isclose(coverage, 1.0)
    assert samples > 0


def test_mechanical_and_fusion_probabilities():
    assert movable_probability(0.019) == 0.0
    assert math.isclose(movable_probability(0.02), 0.60)
    assert math.isclose(movable_probability(0.05), 1.0)
    assert math.isclose(fusion_probability(1.0, 1.0, 1.0), 1.0)
    assert math.isclose(fusion_probability(0.0, 1.0, 1.0), 0.75)
    assert math.isclose(fusion_probability(1.0, 1.0, 0.0), 0.50)


def test_three_modal_fusion_hard_gates_and_visual_compensation():
    assert three_modal_authorized(
        1.0, 1.0, 1.0, mechanical_positive=True,
        mechanical_action_succeeded=True, arm_returned=True,
    )
    assert three_modal_authorized(
        0.0, 1.0, 1.0, mechanical_positive=True,
        mechanical_action_succeeded=True, arm_returned=True,
    )
    assert not three_modal_authorized(
        0.0, 0.0, 1.0, mechanical_positive=True,
        mechanical_action_succeeded=True, arm_returned=True,
    )
    assert not three_modal_authorized(
        1.0, 1.0, 1.0, mechanical_positive=False,
        mechanical_action_succeeded=True, arm_returned=True,
    )
    assert not three_modal_authorized(
        1.0, 1.0, 1.0, mechanical_positive=True,
        mechanical_action_succeeded=False, arm_returned=True,
    )
    assert not three_modal_authorized(
        1.0, 1.0, 1.0, mechanical_positive=True,
        mechanical_action_succeeded=True, arm_returned=False,
    )


def test_path_corridor_selects_nearest_forward_obstacle():
    result = closest_path_obstacle(
        [(0.8, 0.2), (0.5, 0.5), (1.2, 0.1)],
        [(0.0, 0.0), (2.0, 0.0)], (0.0, 0.0),
    )
    assert result is not None
    assert result.point == (0.8, 0.2)


def test_path_corridor_rejects_behind_lateral_and_far_points():
    assert closest_path_obstacle(
        [(-0.1, 0.0), (0.4, 0.5), (1.6, 0.0)],
        [(0.0, 0.0), (2.0, 0.0)], (0.0, 0.0),
    ) is None


def test_sensor_sync_boundary():
    assert synchronized([1.0, 1.1, 1.2], 0.20)
    assert synchronized([1.0, 1.2 + 5e-7], 0.20)
    assert not synchronized([1.0, 1.201], 0.20)


def test_path_guard_can_detect_before_multimodal_snapshot_is_synchronized():
    obstacle = closest_path_obstacle(
        [(1.40, 0.10)], [(0.0, 0.0), (2.0, 0.0)], (0.0, 0.0),
        trigger_distance=1.50, half_width=0.433,
    )
    assert obstacle is not None
    assert not synchronized([10.0, 10.23, 10.0], 0.20)


def test_pretrigger_heading_error_allows_new_path_alignment_before_guarding():
    path = [(2.50, -2.19), (2.91, -3.72)]
    error = path_tangent_heading_error((2.503, -2.191, -0.447), path)
    assert error is not None
    assert error < -0.80
    assert abs(error) > 0.35


def test_pretrigger_heading_error_arms_guard_once_aligned_and_wraps():
    path = [(0.0, 0.0), (-1.0, -0.02)]
    error = path_tangent_heading_error((0.0, 0.0, math.pi - 0.01), path)
    assert error is not None
    assert abs(error) < 0.05


def test_real_isaac_uses_the_locked_multicomponent_target_band():
    from pathlib import Path
    import yaml

    config = yaml.safe_load(
        Path(__file__).parents[1]
        .joinpath("config", "isaac.yaml")
        .read_text(encoding="utf-8")
    )
    approach = config["obstacle_approach_controller"]["ros__parameters"]
    scan_filter = config["obstacle_traversal_scan_filter"]["ros__parameters"]
    assert approach["target_band_half_width_m"] == 0.433
    assert approach["before_obstacle_length_m"] == 0.45
    assert approach["after_obstacle_length_m"] == 0.80
    assert approach["rear_negative_one_is_no_return"] is True
    assert scan_filter["target_band_half_width_m"] == 0.433

    scout = yaml.safe_load(
        Path(__file__).parents[1]
        .joinpath("config", "scout.yaml")
        .read_text(encoding="utf-8")
    )
    assert scout["obstacle_approach_controller"]["ros__parameters"][
        "rear_negative_one_is_no_return"
    ] is False


def test_startup_sensor_readiness_does_not_require_a_navigation_path():
    received = {"scan": 9.9, "image": 9.8, "camera": 9.85}
    assert traversal_sensor_inputs_ready(received, now=10.0, freshness=0.5)
    assert not traversal_sensor_inputs_ready(
        {"scan": 9.9, "image": 9.8}, now=10.0, freshness=0.5
    )


def test_robot_footprint_filter_removes_raw_self_returns():
    points = np.array([
        [0.20, -0.15, 0.17],
        [-0.35, 0.0, 0.17],
        [0.50, 0.0, 0.17],
        [0.0, 0.50, 0.17],
    ])
    assert outside_footprint_mask(
        points, footprint_length=0.62, footprint_width=0.586, padding=0.05
    ).tolist() == [False, False, True, True]


def test_active_track_reassociates_in_map_space_after_path_view_changes():
    points = np.array([[0.2, 0.2, 0.0], [1.8, 1.8, 0.0], [1.1, 1.2, 0.0]])
    assert nearest_tracked_point(points, (1.0, 1.0), 0.5) == 2
    assert nearest_tracked_point(points, (3.0, 3.0), 0.5) is None


def test_external_clearance_excludes_lidar_self_echoes_and_uses_body_edge():
    # LiDAR at x=0.21: 0.15 m returns to each side are inside the Scout body.
    # The forward 0.60 m return is at base x=0.81, hence 0.50 m from x=0.31.
    assert math.isclose(
        minimum_external_scan_clearance(
            [0.15, 0.60, 0.15],
            -math.pi / 2.0,
            math.pi / 2.0,
            0.01,
            5.0,
            lidar_offset_x=0.21,
            lidar_offset_y=0.0,
            footprint_length=0.62,
            footprint_width=0.586,
        ),
        0.50,
    )


def test_external_clearance_returns_none_when_only_self_echoes_are_visible():
    assert minimum_external_scan_clearance(
        [0.15, 0.15],
        -math.pi / 2.0,
        math.pi,
        0.01,
        5.0,
        lidar_offset_x=0.21,
        lidar_offset_y=0.0,
        footprint_length=0.62,
        footprint_width=0.586,
    ) is None


def test_lidar_projection_clips_roi():
    points = np.array([[-0.1, -0.1, 1.0], [0.1, 0.1, 1.0]])
    camera = np.array([[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]])
    assert project_roi(points, np.eye(4), camera, 100, 100, padding=2) == (38, 38, 25, 25)


def test_projected_roi_expands_around_cluster_and_clips_to_image():
    assert expand_roi(
        (100, 97, 186, 202), 640, 480,
        minimum_width=560, minimum_height=480,
    ) == (0, 0, 560, 480)
    assert expand_roi(
        (580, 200, 40, 40), 640, 480,
        minimum_width=320, minimum_height=320,
    ) == (320, 60, 320, 320)


def test_scan_filter_returns_complete_cluster_without_fraction_or_sector_caps():
    ranges = [math.inf] * 1000
    ranges[300:700] = [1.0 + (index % 2) * 0.01 for index in range(400)]
    indices = scan_cluster_indices(ranges, -math.pi, 2 * math.pi / 1000, 0.0, 1.0)
    assert indices == list(range(300, 700))
    assert len(indices) > 0.15 * len(ranges)
    assert (len(indices) - 1) * 2 * math.pi / 1000 > math.radians(45.0)


def test_authorized_path_corridor_removes_disjoint_curtain_components():
    points = np.array([
        [0.70, -0.20],
        [0.85, -0.10],
        [math.nan, math.nan],
        [1.05, 0.15],
        [1.45, 0.05],
        [1.00, 0.60],
        [0.40, 0.00],
        [1.90, 0.00],
    ])
    assert path_corridor_indices(
        points, [(0.0, 0.0), (2.0, 0.0)], (1.0, 0.0),
        half_width=0.50, before_obstacle=0.45, after_obstacle=0.80,
    ) == [0, 1, 3, 4]


def test_target_band_uses_all_curtain_components_and_scout_front_edge():
    points_map = np.array([
        [0.51, -0.08, 0.0],
        [0.53, 0.09, 0.0],
        [0.80, -0.15, 0.0],
        [0.60, 0.70, 0.0],
    ])
    points_robot = points_map.copy()
    observation = target_band_observation(
        points_map,
        points_robot,
        [(0.0, 0.0), (2.0, 0.0)],
        (0.55, 0.0),
        (0.0, 0.0, 0.0),
        footprint_length=0.62,
        footprint_width=0.586,
    )
    assert observation is not None
    assert observation.target_indices == (0, 1, 2)
    assert observation.external_indices == (3,)
    assert math.isclose(observation.clearance, 0.20, abs_tol=1e-9)
    assert abs(observation.bearing) < 0.02


def test_target_band_excludes_other_obstacle_from_target_clearance():
    points_map = np.array([[0.70, 0.0], [0.40, 0.60]])
    observation = target_band_observation(
        points_map,
        points_map,
        [(0.0, 0.0), (2.0, 0.0)],
        (0.70, 0.0),
        (0.0, 0.0, 0.0),
        footprint_length=0.62,
        footprint_width=0.586,
    )
    assert observation is not None
    assert observation.target_indices == (0,)
    assert observation.external_indices == (1,)
    assert math.isclose(observation.clearance, 0.39, abs_tol=1e-9)


def test_authorized_path_corridor_follows_a_bent_route_and_keeps_side_walls():
    points = np.array([
        [0.90, 0.10],
        [1.05, 0.35],
        [1.10, 0.90],
        [1.60, 1.00],
        [0.35, 0.65],
    ])
    assert path_corridor_indices(
        points, [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)], (1.0, 0.5),
        half_width=0.50, before_obstacle=0.45, after_obstacle=0.80,
    ) == [0, 1, 2]


def test_authorized_path_corridor_requires_a_usable_path():
    assert path_corridor_indices(
        np.array([[1.0, 0.0]]), [(0.0, 0.0)], (1.0, 0.0),
        half_width=0.50, before_obstacle=0.45, after_obstacle=0.80,
    ) == []


def test_scan_filter_rejects_track_jump():
    ranges = [math.inf] * 100
    ranges[50] = 2.0
    assert scan_cluster_indices(ranges, -1.0, 0.02, 0.0, 1.0) == []


def test_scan_cluster_cannot_chain_across_a_smooth_range_ramp():
    ranges = [math.inf] * 100
    for index in range(45, 60):
        ranges[index] = 1.0 + 0.1 * (index - 50)
    indices = scan_cluster_indices(
        ranges, -1.0, 0.02, 0.0, 1.5,
        angular_expansion=0.0, edge_points=0,
    )
    assert indices
    assert max(abs(ranges[index] - 1.5) for index in indices) <= 0.36


def test_scan_cluster_expansion_cannot_cross_invalid_returns_into_a_wall():
    ranges = [math.inf] * 100
    ranges[40:48] = [0.25] * 8
    ranges[48] = -1.0
    ranges[49:70] = [3.5] * 21
    indices = scan_cluster_indices(
        ranges, -1.0, 0.02, -0.13, 0.25,
        range_tolerance=0.12, angular_expansion=0.08, edge_points=3,
    )
    assert indices == list(range(40, 48))
    assert all(ranges[index] > 0.0 for index in indices)


def test_scan_cluster_expansion_keeps_nearby_irregular_object_edges():
    ranges = [math.inf] * 100
    ranges[45:50] = [1.0, 1.0, 1.0, 1.22, 1.24]
    indices = scan_cluster_indices(
        ranges, -1.0, 0.02, -0.06, 1.0,
        range_tolerance=0.12, angular_expansion=0.08, edge_points=3,
    )
    assert indices == list(range(45, 50))


def test_velocity_gate_policy_is_fail_closed_only_while_needed():
    assert velocity_gate_mode(
        1, status_stale=False, filter_active=False, filter_matches=False
    ) == "STOP"
    assert velocity_gate_mode(2, status_stale=False, filter_active=False, filter_matches=False) == "STOP"
    assert velocity_gate_mode(3, status_stale=False, filter_active=True, filter_matches=True) == "LIMIT"
    assert velocity_gate_mode(3, status_stale=False, filter_active=False, filter_matches=False) == "STOP"
    assert velocity_gate_mode(5, status_stale=False, filter_active=False, filter_matches=False) == "NORMAL"
    assert velocity_gate_mode(5, status_stale=False, filter_active=True, filter_matches=True) == "STOP"
    assert velocity_gate_mode(6, status_stale=False, filter_active=True, filter_matches=True) == "STOP"
    assert velocity_gate_mode(4, status_stale=True, filter_active=True, filter_matches=True) == "STOP"
    assert velocity_gate_mode(4, status_stale=True, filter_active=False, filter_matches=False) == "STOP"
    assert velocity_gate_mode(None, status_stale=True, filter_active=True, filter_matches=False) == "STOP"
    assert velocity_gate_mode(7, status_stale=False, filter_active=False, filter_matches=False) == "APPROACH"
    assert velocity_gate_mode(10, status_stale=False, filter_active=False, filter_matches=False) == "STOP"
    assert velocity_gate_mode(7, status_stale=True, filter_active=False, filter_matches=False) == "STOP"


def test_authorized_traversal_follows_locked_path_not_neupan_avoidance_turn():
    command = traversal_path_command(
        (0.0, 0.0, 0.0),
        [(0.0, 0.0), (2.0, 0.0)],
        0.40,
    )

    assert command.linear == 0.15
    assert abs(command.angular) < 1e-9
    assert not command.stopped


def test_authorized_traversal_preserves_neupan_stop_and_path_deviation_stop():
    stopped = traversal_path_command(
        (0.0, 0.0, 0.0), [(0.0, 0.0), (2.0, 0.0)], 0.0
    )
    deviated = traversal_path_command(
        (0.0, 0.20, 0.0), [(0.0, 0.0), (2.0, 0.0)], 0.40
    )

    assert stopped.linear == 0.0 and stopped.angular == 0.0
    assert deviated.stopped and deviated.linear == 0.0
