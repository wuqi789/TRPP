import math

from visualization_msgs.msg import Marker

from obstacle_traversal.system_diagnostics import pose_alignment


def test_pose_alignment_matches_marker_to_tf():
    marker = Marker()
    marker.pose.position.x = 2.0
    marker.pose.position.y = -1.0
    marker.pose.orientation.z = math.sin(0.4 / 2.0)
    marker.pose.orientation.w = math.cos(0.4 / 2.0)

    translation, yaw = pose_alignment((2.03, -1.04, 0.35), marker)

    assert math.isclose(translation, 0.05)
    assert math.isclose(yaw, 0.05)


def test_pose_alignment_wraps_yaw_at_pi():
    marker = Marker()
    marker.pose.orientation.z = math.sin((-math.pi + 0.02) / 2.0)
    marker.pose.orientation.w = math.cos((-math.pi + 0.02) / 2.0)

    _, yaw = pose_alignment((0.0, 0.0, math.pi - 0.02), marker)

    assert math.isclose(yaw, 0.04)
