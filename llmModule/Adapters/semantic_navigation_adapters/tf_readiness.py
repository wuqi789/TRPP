"""Small replaceable readiness port for the execution-frame transform."""

from __future__ import annotations

from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class TfReadiness:
    def __init__(
        self,
        node,
        *,
        global_frame: str = "odom",
        robot_frame: str = "base_link",
    ) -> None:
        self.global_frame = global_frame
        self.robot_frame = robot_frame
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, node, spin_thread=False)

    @property
    def ready(self) -> bool:
        try:
            return bool(
                self._buffer.can_transform(
                    self.global_frame,
                    self.robot_frame,
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
            )
        except Exception:
            return False
