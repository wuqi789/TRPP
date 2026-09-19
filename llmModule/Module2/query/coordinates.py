"""Convert static USD-stage poses into the robot's navigation frame."""

from __future__ import annotations

import math
from typing import Any


def relative_pose(pose: Any, origin: Any) -> tuple[float, float, float]:
    """Express a world pose relative to an origin pose using SE(2)."""
    dx = float(pose.x) - float(origin.x)
    dy = float(pose.y) - float(origin.y)
    yaw = float(origin.theta)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    x = cosine * dx + sine * dy
    y = -sine * dx + cosine * dy
    theta = math.atan2(
        math.sin(float(pose.theta) - yaw),
        math.cos(float(pose.theta) - yaw),
    )
    return x, y, theta
