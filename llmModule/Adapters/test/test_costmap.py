from types import SimpleNamespace

from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid

from semantic_navigation_adapters import CostmapCache


class Node:
    def __init__(self):
        self.subscriptions = {}
        self.clock = Clock()

    def create_subscription(self, _message_type, topic, callback, _qos):
        self.subscriptions[topic] = callback

    def get_clock(self):
        return self.clock


class Clock:
    def __init__(self, seconds=10.0):
        self.seconds = seconds

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.seconds * 1e9))


class WallClock:
    def __init__(self):
        self.seconds = 0.0

    def __call__(self):
        return self.seconds


def grid():
    value = OccupancyGrid()
    value.info.resolution = 1.0
    value.info.width = 4
    value.info.height = 3
    value.data = [0] * 12
    return value


def test_incremental_update_refreshes_and_mutates_cached_costmap():
    node = Node()
    cache = CostmapCache(node, "/global_costmap/costmap", robot_radius=0.0)
    assert set(node.subscriptions) == {
        "/global_costmap/costmap",
        "/global_costmap/costmap_updates",
    }

    node.subscriptions["/global_costmap/costmap"](grid())
    node.clock.seconds += 3.0
    assert not cache.ready

    update = OccupancyGridUpdate()
    update.x = 1
    update.y = 1
    update.width = 2
    update.height = 1
    update.data = [99, 0]
    node.subscriptions["/global_costmap/costmap_updates"](update)

    assert cache.ready
    result = cache.validate_pose(1.0, 1.0)
    assert not result.valid
    assert result.code == "GEOMETRY_OCCUPIED"


def test_invalid_increment_does_not_make_stale_costmap_fresh():
    node = Node()
    cache = CostmapCache(node, "/costmap", maximum_age=0.1)
    node.subscriptions["/costmap"](grid())
    node.clock.seconds += 1.0

    invalid = OccupancyGridUpdate()
    invalid.x = 3
    invalid.y = 2
    invalid.width = 2
    invalid.height = 2
    invalid.data = [0] * 4
    node.subscriptions["/costmap_updates"](invalid)

    assert not cache.ready


def test_low_real_time_factor_uses_ros_age_not_wall_age():
    node = Node()
    wall = WallClock()
    cache = CostmapCache(node, "/costmap", monotonic=wall)
    node.subscriptions["/costmap"](grid())

    wall.seconds += 4.0
    node.clock.seconds += 0.5

    assert cache.ready


def test_stalled_ros_clock_has_specific_failure():
    node = Node()
    wall = WallClock()
    cache = CostmapCache(
        node, "/costmap", monotonic=wall, clock_stall_timeout=5.0
    )
    node.subscriptions["/costmap"](grid())

    wall.seconds += 5.1

    assert cache.status.code == "SIM_TIME_STALLED"


def test_ros_clock_jump_invalidates_old_costmap():
    node = Node()
    cache = CostmapCache(node, "/costmap")
    node.subscriptions["/costmap"](grid())

    node.clock.seconds = 1.0

    assert cache.status.code == "COSTMAP_UNAVAILABLE"


def test_inflated_inscribed_band_is_not_applied_twice_to_footprint():
    node = Node()
    cache = CostmapCache(
        node, "/costmap", robot_radius=1.1, occupied_threshold=99
    )
    message = grid()
    # The center is free. An adjacent cost 99 is Nav2's already-inflated
    # inscribed band and must not be expanded by the robot radius again.
    message.data[1 * message.info.width + 2] = 99
    node.subscriptions["/costmap"](message)

    assert cache.validate_pose(1.0, 1.0).valid


def test_lethal_cell_inside_footprint_still_fails_geometry():
    node = Node()
    cache = CostmapCache(node, "/costmap", robot_radius=1.1)
    message = grid()
    message.data[1 * message.info.width + 2] = 100
    node.subscriptions["/costmap"](message)

    result = cache.validate_pose(1.0, 1.0)

    assert not result.valid
    assert result.code == "GEOMETRY_OCCUPIED"
