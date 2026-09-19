import math

from nav_msgs.msg import OccupancyGrid

from module3_ros_interface.map_normalizer import normalize_occupancy_grid


def grid(width, height, data, yaw=0.0):
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.resolution = 1.0
    message.info.width = width
    message.info.height = height
    message.info.origin.orientation.z = math.sin(yaw / 2.0)
    message.info.origin.orientation.w = math.cos(yaw / 2.0)
    message.data = data
    return message


def test_axis_aligned_map_preserves_cells():
    output = normalize_occupancy_grid(grid(2, 2, [0, 100, 25, 50]))

    assert output.header.frame_id == "odom"
    assert output.info.width == 2
    assert output.info.height == 2
    assert list(output.data) == [0, 100, 25, 50]
    assert output.info.origin.orientation.w == 1.0


def test_quarter_turn_is_resampled_into_axis_aligned_grid():
    output = normalize_occupancy_grid(
        grid(2, 1, [0, 100], yaw=math.pi / 2.0)
    )

    assert output.info.width == 1
    assert output.info.height == 2
    assert math.isclose(output.info.origin.position.x, -1.0, abs_tol=1e-9)
    assert math.isclose(output.info.origin.position.y, 0.0, abs_tol=1e-9)
    assert list(output.data) == [0, 100]


def test_invalid_data_size_is_rejected():
    message = grid(2, 2, [0])

    try:
        normalize_occupancy_grid(message)
    except ValueError as exc:
        assert "data size" in str(exc)
    else:
        raise AssertionError("invalid grid was accepted")
