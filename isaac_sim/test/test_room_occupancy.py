import math

from isaac_sim.room_occupancy import (
    NAVIGATION_MAX_Z,
    OPEN_KITCHEN_DOOR_CENTER_Y,
    OPEN_KITCHEN_DOOR_HALF_WIDTH,
    TRAVERSABLE_DOOR_LEAVES,
    intersects_navigation_height,
    is_blocking_other_prim,
    solid_spans_around_openings,
    world_origin_in_start_frame,
)


def test_known_open_door_leaves_do_not_block_the_occupancy_map():
    assert TRAVERSABLE_DOOR_LEAVES == {"door_0000", "door_0001"}
    assert not is_blocking_other_prim("door_0000")
    assert not is_blocking_other_prim("door_0001")


def test_closed_door_leaves_remain_structural_obstacles():
    assert is_blocking_other_prim("door_0002")
    assert is_blocking_other_prim("door_0003")
    assert is_blocking_other_prim("door_0004")


def test_floor_level_door_sills_never_close_a_portal():
    for index in range(5):
        assert not is_blocking_other_prim(f"doorsill_{index:04d}")


def test_non_door_other_prims_are_not_selected_as_structural_proxies():
    assert not is_blocking_other_prim("window_0000")
    assert not is_blocking_other_prim("painting_0000")


def test_combined_wall_is_split_around_both_traversable_openings():
    curtain = (-3.624, -2.784)
    kitchen = (
        OPEN_KITCHEN_DOOR_CENTER_Y - OPEN_KITCHEN_DOOR_HALF_WIDTH,
        OPEN_KITCHEN_DOOR_CENTER_Y + OPEN_KITCHEN_DOOR_HALF_WIDTH,
    )
    assert solid_spans_around_openings(-4.5, 1.1, (curtain, kitchen)) == (
        (-4.5, curtain[0]),
        (curtain[1], kitchen[0]),
        (kitchen[1], 1.1),
    )


def test_overhead_lintel_is_not_a_ground_plane_obstacle():
    assert intersects_navigation_height(0.0, 3.3)
    assert not intersects_navigation_height(NAVIGATION_MAX_Z, 3.3)


def test_world_origin_transform_places_launch_pose_at_local_origin():
    start_x, start_y, start_yaw = 7.7, -4.0, 1.0
    tx, ty, yaw = world_origin_in_start_frame(start_x, start_y, start_yaw)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    local_x = tx + cosine * start_x - sine * start_y
    local_y = ty + sine * start_x + cosine * start_y
    assert math.isclose(local_x, 0.0, abs_tol=1e-9)
    assert math.isclose(local_y, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, -1.0, abs_tol=1e-9)
