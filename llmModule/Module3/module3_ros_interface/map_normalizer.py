"""Normalize a rotated OccupancyGrid into an axis-aligned validation map."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _yaw(message: OccupancyGrid) -> float:
    orientation = message.info.origin.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )


def normalize_occupancy_grid(
    source: OccupancyGrid, output_frame: str = "odom"
) -> OccupancyGrid:
    info = source.info
    resolution = float(info.resolution)
    if resolution <= 0.0 or info.width <= 0 or info.height <= 0:
        raise ValueError("Source occupancy grid metadata is invalid")
    if len(source.data) != info.width * info.height:
        raise ValueError("Source occupancy grid data size is invalid")

    yaw = _yaw(source)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    origin_x = float(info.origin.position.x)
    origin_y = float(info.origin.position.y)
    extent_x = info.width * resolution
    extent_y = info.height * resolution
    corners = [
        (
            origin_x + cosine * local_x - sine * local_y,
            origin_y + sine * local_x + cosine * local_y,
        )
        for local_x, local_y in (
            (0.0, 0.0),
            (extent_x, 0.0),
            (0.0, extent_y),
            (extent_x, extent_y),
        )
    ]
    minimum_x = min(value[0] for value in corners)
    minimum_y = min(value[1] for value in corners)
    maximum_x = max(value[0] for value in corners)
    maximum_y = max(value[1] for value in corners)
    width = max(1, math.ceil((maximum_x - minimum_x) / resolution - 1e-9))
    height = max(1, math.ceil((maximum_y - minimum_y) / resolution - 1e-9))

    output = OccupancyGrid()
    output.header.stamp = source.header.stamp
    output.header.frame_id = output_frame
    output.info.map_load_time = source.info.map_load_time
    output.info.resolution = resolution
    output.info.width = width
    output.info.height = height
    output.info.origin.position.x = minimum_x
    output.info.origin.position.y = minimum_y
    output.info.origin.position.z = source.info.origin.position.z
    output.info.origin.orientation.w = 1.0

    data = [100] * (width * height)
    for output_y in range(height):
        world_y = minimum_y + (output_y + 0.5) * resolution
        for output_x in range(width):
            world_x = minimum_x + (output_x + 0.5) * resolution
            delta_x = world_x - origin_x
            delta_y = world_y - origin_y
            local_x = cosine * delta_x + sine * delta_y
            local_y = -sine * delta_x + cosine * delta_y
            source_x = math.floor(local_x / resolution)
            source_y = math.floor(local_y / resolution)
            if 0 <= source_x < info.width and 0 <= source_y < info.height:
                data[output_y * width + output_x] = int(
                    source.data[source_y * info.width + source_x]
                )
    output.data = data
    return output


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


class ValidationMapNormalizer(Node):
    def __init__(self) -> None:
        super().__init__("validation_map_normalizer")
        self.declare_parameter("source_topic", "/map")
        self.declare_parameter("output_topic", "/semantic_validation/map")
        self.declare_parameter("output_frame", "odom")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._source = None
        self._publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("output_topic").value), qos
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("source_topic").value),
            self._on_map,
            qos,
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        if self._source is not None and _same_static_map(self._source, message):
            return
        self._source = message
        try:
            output = normalize_occupancy_grid(
                message, str(self.get_parameter("output_frame").value)
            )
        except ValueError:
            self.get_logger().error("Map normalization rejected the input")
            return
        self._publisher.publish(output)
        self.get_logger().info(
            f"Normalized rotated map {message.info.width}x{message.info.height} "
            f"yaw={_yaw(message):.4f} to {output.info.width}x{output.info.height} "
            f"in {output.header.frame_id}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ValidationMapNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
