"""Per-request transient keepout-mask publisher."""

from __future__ import annotations

import math
import threading
import time

from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class KeepoutMaskManager:
    def __init__(
        self,
        node,
        *,
        map_topic: str = "/map",
        mask_topic: str = "/semantic_navigation/keepout_mask",
        info_topic: str = "/semantic_navigation/keepout_filter_info",
        settle_seconds: float = 0.25,
    ) -> None:
        self._node = node
        self.mask_topic = mask_topic
        self.settle_seconds = float(settle_seconds)
        self._lock = threading.Lock()
        self._map: OccupancyGrid | None = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(OccupancyGrid, map_topic, self._on_map, qos)
        self._mask_publisher = node.create_publisher(OccupancyGrid, mask_topic, qos)
        self._info_publisher = node.create_publisher(CostmapFilterInfo, info_topic, qos)

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._map is not None

    def _on_map(self, message: OccupancyGrid) -> None:
        with self._lock:
            previous = self._map
            changed = previous is None or not self._same_static_map(previous, message)
            self._map = message
        if changed:
            self.clear(wait=False)

    def apply(self, constraints, *, wait: bool = True) -> bool:
        with self._lock:
            source = self._map
        if source is None:
            return False
        mask = OccupancyGrid()
        mask.header.frame_id = source.header.frame_id or "odom"
        mask.header.stamp = self._node.get_clock().now().to_msg()
        mask.info = source.info
        data = [0] * (source.info.width * source.info.height)
        for constraint in constraints:
            if str(constraint.type).casefold() != "avoid":
                continue
            self._paint_circle(
                data,
                source,
                float(constraint.pose.pose.position.x),
                float(constraint.pose.pose.position.y),
                float(constraint.radius),
            )
        mask.data = data
        info = CostmapFilterInfo()
        info.header = mask.header
        info.type = 0
        info.filter_mask_topic = self.mask_topic
        info.base = 0.0
        info.multiplier = 1.0
        self._info_publisher.publish(info)
        self._mask_publisher.publish(mask)
        if wait and self.settle_seconds > 0:
            time.sleep(self.settle_seconds)
        return True

    def clear(self, *, wait: bool = True) -> bool:
        return self.apply([], wait=wait)

    @staticmethod
    def _same_static_map(left: OccupancyGrid, right: OccupancyGrid) -> bool:
        left_origin = left.info.origin
        right_origin = right.info.origin
        return (
            left.header.frame_id == right.header.frame_id
            and left.info.width == right.info.width
            and left.info.height == right.info.height
            and left.info.resolution == right.info.resolution
            and left_origin.position == right_origin.position
            and left_origin.orientation == right_origin.orientation
            and left.data == right.data
        )

    @staticmethod
    def _paint_circle(data: list[int], grid: OccupancyGrid, x: float, y: float, radius: float) -> None:
        resolution = grid.info.resolution
        center_x = math.floor((x - grid.info.origin.position.x) / resolution)
        center_y = math.floor((y - grid.info.origin.position.y) / resolution)
        cells = max(0, math.ceil(radius / resolution))
        for offset_y in range(-cells, cells + 1):
            for offset_x in range(-cells, cells + 1):
                if math.hypot(offset_x, offset_y) * resolution > radius:
                    continue
                cell_x, cell_y = center_x + offset_x, center_y + offset_y
                if 0 <= cell_x < grid.info.width and 0 <= cell_y < grid.info.height:
                    data[cell_y * grid.info.width + cell_x] = 100
