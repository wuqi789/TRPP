import io
import math

from PIL import Image
import pytest

from route_planning.grid import GridMap, GridPoint, RouteValidationError


def grid(data=None, **kwargs):
    return GridMap(
        width=kwargs.pop("width", 7),
        height=kwargs.pop("height", 5),
        resolution=kwargs.pop("resolution", 1.0),
        origin_x=kwargs.pop("origin_x", 0.0),
        origin_y=kwargs.pop("origin_y", 0.0),
        data=data or [0] * 35,
        inflation_radius=kwargs.pop("inflation_radius", 0.0),
        scale=kwargs.pop("scale", 4),
        **kwargs,
    )


def test_world_pixel_round_trip_uses_top_left_image_origin():
    value = grid()
    bottom_left = value.cell_to_world(0, 0)
    assert value.world_to_pixel(bottom_left) == (2, 18)
    assert value.pixel_to_world(2, 18) == bottom_left
    assert value.pixel_to_world(26, 2) == value.cell_to_world(6, 4)


def test_rotated_map_origin_round_trips_world_coordinates():
    value = grid(origin_x=-1.0, origin_y=-2.0, origin_yaw=math.pi / 2.0)
    point = value.cell_to_world(2, 1)
    assert point.x == pytest.approx(-2.5)
    assert point.y == pytest.approx(0.5)
    assert value.world_to_cell(point) == (2, 1)
    assert value.pixel_to_world(*value.world_to_pixel(point)) == point


def test_unknown_occupied_and_clearance_cells_are_blocked():
    data = [0] * 35
    data[2 * 7 + 3] = -1
    value = grid(data, resolution=0.05, inflation_radius=0.10)
    for cell in ((3, 2), (1, 2), (5, 2), (3, 0), (3, 4)):
        assert not value.is_free(value.cell_to_world(*cell))


def test_avoid_region_is_rendered_and_excluded_from_free_space():
    value = grid(avoid_regions=[(2.5, 2.5, 0.4)])
    point = GridPoint(2.5, 2.5)
    with pytest.raises(RouteValidationError, match="not in free space"):
        value.validate_point(point)
    image = Image.open(
        io.BytesIO(value.render(GridPoint(0.5, 0.5), GridPoint(6.5, 4.5)))
    )
    assert image.getpixel(value.world_to_pixel(point)) == (230, 126, 34)


def test_pushable_region_is_blue_but_remains_blocked_for_waypoints():
    data = [0] * 35
    data[2 * 7 + 3] = 100
    value = grid(
        data,
        pushable_regions=[(3.5, 2.5, 1.0, 1.0)],
    )
    point = GridPoint(3.5, 2.5)
    image = Image.open(
        io.BytesIO(value.render(GridPoint(0.5, 0.5), GridPoint(6.5, 4.5)))
    )
    assert image.getpixel(value.world_to_pixel(point)) == (32, 122, 220)
    with pytest.raises(RouteValidationError, match="not in free space"):
        value.validate_point(point)


def test_pushable_region_fills_occupancy_gap_but_remains_waypoint_blocked():
    value = grid(pushable_regions=[(3.5, 2.5, 1.0, 1.0)])
    point = GridPoint(3.5, 2.5)
    image = Image.open(
        io.BytesIO(value.render(GridPoint(0.5, 0.5), GridPoint(6.5, 4.5)))
    )
    assert image.getpixel(value.world_to_pixel(point)) == (32, 122, 220)
    with pytest.raises(RouteValidationError, match="not in free space"):
        value.validate_point(point)


def test_connected_component_does_not_cross_four_connected_wall():
    data = [0] * 35
    for y in range(5):
        data[y * 7 + 3] = 100
    value = grid(data)
    component, seed = value.connected_component(GridPoint(1.5, 2.5), 0.5)
    assert seed == GridPoint(1.5, 2.5)
    assert (2, 2) in component
    assert (4, 2) not in component


def test_blocked_point_snaps_deterministically_to_connected_free_cell():
    data = [0] * 35
    data[2 * 7 + 3] = 100
    value = grid(data)
    component, _ = value.connected_component(GridPoint(1.5, 2.5), 0.5)
    snapped = value.snap_to_component(
        GridPoint(3.5, 2.5), component, 1.1, "waypoint[0]"
    )
    assert snapped == GridPoint(3.5, 1.5)


def test_disconnected_point_snaps_only_within_configured_distance():
    data = [0] * 35
    for y in range(5):
        data[y * 7 + 3] = 100
    value = grid(data)
    component, _ = value.connected_component(GridPoint(1.5, 2.5), 0.5)
    point = GridPoint(4.5, 2.5)
    with pytest.raises(RouteValidationError, match="within 0.50 m"):
        value.snap_to_component(point, component, 0.5, "waypoint[0]")
    assert value.snap_to_component(
        point, component, 2.1, "waypoint[0]"
    ) == GridPoint(2.5, 2.5)


def test_start_inside_clearance_uses_nearest_free_connectivity_seed():
    data = [0] * 81
    data[4 * 9 + 4] = 100
    value = GridMap(
        width=9,
        height=9,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        data=data,
        inflation_radius=1.0,
        scale=4,
    )
    component, seed = value.connected_component(GridPoint(5.5, 4.5), 2.0)
    assert seed == GridPoint(5.5, 3.5)
    assert value.world_to_cell(seed) in component


def test_out_of_bounds_points_never_snap():
    value = grid()
    component, _ = value.connected_component(GridPoint(1.5, 1.5), 0.5)
    with pytest.raises(RouteValidationError, match="outside"):
        value.pixel_to_world(value.pixel_width, 0)
    with pytest.raises(RouteValidationError, match="outside"):
        value.snap_to_component(GridPoint(-0.1, 1.0), component, 5.0)


def test_grid_no_longer_exposes_segment_collision_validation():
    value = grid()
    assert not hasattr(value, "validate_segment")
    assert not hasattr(value, "validate_route")
    assert not hasattr(value, "find_start_egress")
