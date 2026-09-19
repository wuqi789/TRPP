"""Thread-safe validation against a live Nav2 OccupancyGrid."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


@dataclass(frozen=True)
class CostmapValidation:
    valid: bool
    code: str = ""
    message: str = ""


class CostmapCache:
    def __init__(
        self,
        node,
        topic: str,
        *,
        update_topic: str | None = None,
        maximum_age: float = 2.0,
        clock_stall_timeout: float = 5.0,
        occupied_threshold: int = 99,
        robot_radius: float = 0.333,
        monotonic=time.monotonic,
    ) -> None:
        self._node = node
        self.maximum_age = float(maximum_age)
        self.clock_stall_timeout = float(clock_stall_timeout)
        self.occupied_threshold = int(occupied_threshold)
        self.robot_radius = float(robot_radius)
        self._monotonic = monotonic
        self._lock = threading.RLock()
        self._changed = threading.Condition(self._lock)
        self._message: OccupancyGrid | None = None
        self._received_ros = 0.0
        self._clock_ros = 0.0
        self._clock_progress_wall = self._monotonic()
        self._sequence = 0
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(OccupancyGrid, topic, self._on_costmap, qos)
        node.create_subscription(
            OccupancyGridUpdate,
            update_topic or f"{topic}_updates",
            self._on_costmap_update,
            qos,
        )

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._status_locked().valid

    @property
    def sequence(self) -> int:
        with self._lock:
            return self._sequence

    @property
    def status(self) -> CostmapValidation:
        with self._lock:
            return self._status_locked()

    def wait_for_update(self, after_sequence: int, timeout: float) -> bool:
        deadline = self._monotonic() + max(0.0, float(timeout))
        with self._changed:
            while self._sequence <= after_sequence:
                remaining = deadline - self._monotonic()
                if remaining <= 0.0:
                    return False
                self._changed.wait(min(remaining, 0.1))
            return self._status_locked().valid

    def _on_costmap(self, message: OccupancyGrid) -> None:
        with self._changed:
            now_ros = self._observe_clock_locked()
            self._message = message
            self._received_ros = now_ros
            self._sequence += 1
            self._changed.notify_all()

    def _on_costmap_update(self, update: OccupancyGridUpdate) -> None:
        with self._changed:
            message = self._message
            if message is None:
                return
            x = int(update.x)
            y = int(update.y)
            width = int(update.width)
            height = int(update.height)
            if (
                x < 0
                or y < 0
                or width <= 0
                or height <= 0
                or x + width > message.info.width
                or y + height > message.info.height
                or len(update.data) != width * height
            ):
                return
            for row in range(height):
                source_start = row * width
                target_start = (y + row) * message.info.width + x
                message.data[target_start : target_start + width] = update.data[
                    source_start : source_start + width
                ]
            message.header.stamp = update.header.stamp
            self._received_ros = self._observe_clock_locked()
            self._sequence += 1
            self._changed.notify_all()

    def validate_pose(self, x: float, y: float) -> CostmapValidation:
        with self._lock:
            status = self._status_locked()
            message = self._message
            if not status.valid:
                return status
            assert message is not None
            info = message.info
            if info.resolution <= 0.0 or info.width == 0 or info.height == 0:
                return CostmapValidation(
                    False, "COSTMAP_INVALID", "Costmap metadata is invalid"
                )
            radius_cells = max(1, math.ceil(self.robot_radius / info.resolution))
            center_x = math.floor((x - info.origin.position.x) / info.resolution)
            center_y = math.floor((y - info.origin.position.y) / info.resolution)
            if not (0 <= center_x < info.width and 0 <= center_y < info.height):
                return CostmapValidation(
                    False,
                    "GEOMETRY_OUT_OF_BOUNDS",
                    "Robot center is outside the costmap",
                )
            center_value = int(
                message.data[center_y * info.width + center_x]
            )
            if center_value < 0:
                return CostmapValidation(
                    False,
                    "GEOMETRY_UNKNOWN",
                    "Robot center is in unknown space",
                )
            if center_value >= self.occupied_threshold:
                return CostmapValidation(
                    False,
                    "GEOMETRY_OCCUPIED",
                    "Robot center is in an occupied or inscribed costmap cell",
                )
            for offset_y in range(-radius_cells, radius_cells + 1):
                for offset_x in range(-radius_cells, radius_cells + 1):
                    if (
                        math.hypot(offset_x, offset_y) * info.resolution
                        > self.robot_radius
                    ):
                        continue
                    cell_x, cell_y = center_x + offset_x, center_y + offset_y
                    if not (0 <= cell_x < info.width and 0 <= cell_y < info.height):
                        return CostmapValidation(
                            False,
                            "GEOMETRY_OUT_OF_BOUNDS",
                            "Robot footprint leaves the costmap",
                        )
                    value = int(message.data[cell_y * info.width + cell_x])
                    if value < 0:
                        return CostmapValidation(
                            False,
                            "GEOMETRY_UNKNOWN",
                            "Robot footprint enters unknown space",
                        )
                    # This cache consumes Nav2's already-inflated costmap.
                    # Cost 99 is the inscribed inflation band and has already
                    # accounted for the configured robot footprint. Applying
                    # the radius to that band again would double-inflate nearby
                    # walls. Preserve the footprint sweep for true lethal
                    # cells (100), unknown space, and map boundaries.
                    if value >= 100:
                        return CostmapValidation(
                            False,
                            "GEOMETRY_OCCUPIED",
                            "Robot footprint intersects a lethal cell",
                        )
        return CostmapValidation(True)

    def _now_ros(self) -> float:
        return float(self._node.get_clock().now().nanoseconds) / 1e9

    def _observe_clock_locked(self) -> float:
        now_ros = self._now_ros()
        now_wall = self._monotonic()
        if now_ros < self._clock_ros:
            # Isaac can reset /clock when a scene is restarted.  Never reuse a
            # costmap received before that reset.
            self._message = None
            self._received_ros = 0.0
            self._sequence += 1
            self._changed.notify_all()
            self._clock_progress_wall = now_wall
        elif now_ros > self._clock_ros:
            self._clock_progress_wall = now_wall
        self._clock_ros = now_ros
        return now_ros

    def _status_locked(self) -> CostmapValidation:
        now_ros = self._observe_clock_locked()
        if self._message is None:
            return CostmapValidation(
                False, "COSTMAP_UNAVAILABLE", "No current costmap has been received"
            )
        stalled_for = self._monotonic() - self._clock_progress_wall
        if stalled_for > self.clock_stall_timeout:
            return CostmapValidation(
                False,
                "SIM_TIME_STALLED",
                f"ROS time has not advanced for {stalled_for:.2f}s wall time",
            )
        age = now_ros - self._received_ros
        if age < 0.0:
            return CostmapValidation(
                False, "COSTMAP_UNAVAILABLE", "ROS time moved behind the cached costmap"
            )
        if age > self.maximum_age:
            return CostmapValidation(
                False, "COSTMAP_STALE", f"Costmap is {age:.2f}s old in ROS time"
            )
        return CostmapValidation(True)
