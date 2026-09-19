import numpy as np

from builtin_interfaces.msg import Time
from neupan_ros2.visualization_manager import VisualizationManager
from visualization_msgs.msg import Marker


class _Clock:
    class _Now:
        @staticmethod
        def to_msg():
            return Time()

    @staticmethod
    def now():
        return _Clock._Now()


class _Node:
    @staticmethod
    def get_clock():
        return _Clock()


def _manager():
    manager = VisualizationManager.__new__(VisualizationManager)
    manager.node = _Node()
    manager.map_frame = "map"
    manager.marker_size = 0.05
    return manager


def test_nrmp_points_use_one_marker_instead_of_one_cube_per_point():
    output = _manager()._generate_nrmp_markers(
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    )

    assert len(output.markers) == 1
    marker = output.markers[0]
    assert marker.type == Marker.POINTS
    assert marker.ns == "nrmp_points"
    assert marker.header.frame_id == "map"
    assert marker.pose.orientation.w == 1.0
    assert [(point.x, point.y, point.z) for point in marker.points] == [
        (1.0, 4.0, 0.3),
        (2.0, 5.0, 0.3),
        (3.0, 6.0, 0.3),
    ]
