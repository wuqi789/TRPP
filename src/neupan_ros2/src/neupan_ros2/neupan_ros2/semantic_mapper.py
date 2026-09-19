"""Build a voted 3D semantic map from aligned RGB and depth images."""

import colorsys
import math
import time
from typing import Optional

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, Header
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


# The public build exposes only a small, provider-neutral label vocabulary.
# A private deployment may replace the segmenter behind this stable interface.
MOCK_SEMANTIC_CLASSES = (
    "background",
    "obstacle",
    "curtain",
    "surface",
)


def make_palette(size: int) -> np.ndarray:
    """Return stable, high-contrast RGB colors for semantic classes."""
    colors = []
    for index in range(size):
        hue = (index * 0.61803398875) % 1.0
        rgb = colorsys.hsv_to_rgb(hue, 0.72, 0.95)
        colors.append(tuple(round(channel * 255) for channel in rgb))
    return np.asarray(colors, dtype=np.uint8)


def make_polygon_mask(
    height: int, width: int, normalized_points: list[float]
) -> np.ndarray:
    """Rasterize a normalized image polygon used for robot self-filtering."""
    mask = np.zeros((height, width), dtype=bool)
    if not normalized_points:
        return mask
    if len(normalized_points) < 6 or len(normalized_points) % 2:
        raise ValueError(
            "self_mask_polygon must contain at least three x/y pairs"
        )
    points = np.asarray(normalized_points, dtype=np.float32).reshape(-1, 2)
    if np.any(points < 0.0) or np.any(points > 1.0):
        raise ValueError("self_mask_polygon coordinates must be in [0, 1]")
    pixels = np.column_stack(
        (points[:, 0] * (width - 1), points[:, 1] * (height - 1))
    ).round().astype(np.int32)
    cv2.fillPoly(mask.view(np.uint8), [pixels], 1)
    return mask


class VoxelVoteMap:
    """Store one semantic class vote per voxel and input frame."""

    def __init__(self, voxel_size: float, class_count: int) -> None:
        if voxel_size <= 0.0 or class_count < 1:
            raise ValueError("voxel_size and class_count must be positive")
        self.voxel_size = voxel_size
        self.class_count = class_count
        self._votes: dict[tuple[int, int, int], np.ndarray] = {}

    def add_frame(self, points: np.ndarray, labels: np.ndarray) -> int:
        """Fuse a frame after reducing all pixels in each voxel to one vote."""
        if not len(points):
            return 0
        keys = np.floor(points / self.voxel_size).astype(np.int32)
        unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
        packed = inverse.astype(np.int64) * self.class_count + labels
        counts = np.bincount(
            packed, minlength=len(unique_keys) * self.class_count
        ).reshape(len(unique_keys), self.class_count)
        frame_labels = np.argmax(counts, axis=1)
        for key_array, label in zip(unique_keys, frame_labels):
            key = tuple(int(value) for value in key_array)
            votes = self._votes.setdefault(
                key, np.zeros(self.class_count, dtype=np.uint16)
            )
            if votes[label] == np.iinfo(np.uint16).max:
                votes //= 2
            votes[label] += 1
        return len(unique_keys)

    def resolved(
        self, min_votes: int, min_ratio: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return voxel centers, winning classes, vote totals, and ratios."""
        points = []
        labels = []
        totals = []
        ratios = []
        for key, votes in self._votes.items():
            total = int(votes.sum())
            if total < min_votes:
                continue
            label = int(np.argmax(votes))
            ratio = float(votes[label]) / total
            if ratio < min_ratio:
                continue
            points.append((np.asarray(key) + 0.5) * self.voxel_size)
            labels.append(label)
            totals.append(min(total, np.iinfo(np.uint16).max))
            ratios.append(ratio)
        return (
            np.asarray(points, dtype=np.float32).reshape(-1, 3),
            np.asarray(labels, dtype=np.uint16),
            np.asarray(totals, dtype=np.uint16),
            np.asarray(ratios, dtype=np.float32),
        )

    def __len__(self) -> int:
        return len(self._votes)


def build_obstacle_layer(
    points: np.ndarray,
    labels: np.ndarray,
    totals: np.ndarray,
    ratios: np.ndarray,
    occupancy: OccupancyGrid,
    ignored_labels: set[int],
    occupancy_threshold: int,
    snap_radius: float,
    fill_radius: float,
    layer_z: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Collapse voted semantics onto nearby occupied 2D map cells."""
    empty = (
        np.empty((0, 3), dtype=np.float32),
        np.empty(0, dtype=np.uint16),
        np.empty(0, dtype=np.uint16),
        np.empty(0, dtype=np.float32),
        np.empty((0, 2), dtype=np.int32),
    )
    info = occupancy.info
    if (
        not len(points)
        or info.width < 1
        or info.height < 1
        or info.resolution <= 0.0
        or len(occupancy.data) != info.width * info.height
    ):
        return empty

    keep = ~np.isin(labels, tuple(ignored_labels))
    if not np.any(keep):
        return empty
    points = points[keep]
    labels = labels[keep]
    weights = totals[keep].astype(np.float64) * ratios[keep]

    grid = np.asarray(occupancy.data, dtype=np.int16).reshape(
        info.height, info.width
    )
    origin = info.origin
    quaternion = origin.orientation
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    delta_x = points[:, 0] - origin.position.x
    delta_y = points[:, 1] - origin.position.y
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    columns = np.floor(local_x / info.resolution).astype(np.int32)
    rows = np.floor(local_y / info.resolution).astype(np.int32)

    snap_cells = max(0, math.ceil(snap_radius / info.resolution))
    seed_votes: dict[tuple[int, int], float] = {}
    for row, column, x_local, y_local, label, weight in zip(
        rows, columns, local_x, local_y, labels, weights
    ):
        best_cell = None
        best_distance = float("inf")
        if (
            0 <= row < info.height
            and 0 <= column < info.width
            and grid[row, column] >= occupancy_threshold
        ):
            best_cell = int(row * info.width + column)
            best_distance = -1.0
        for row_offset in range(-snap_cells, snap_cells + 1):
            candidate_row = int(row + row_offset)
            if candidate_row < 0 or candidate_row >= info.height:
                continue
            for column_offset in range(-snap_cells, snap_cells + 1):
                candidate_column = int(column + column_offset)
                if candidate_column < 0 or candidate_column >= info.width:
                    continue
                if grid[candidate_row, candidate_column] < occupancy_threshold:
                    continue
                center_x = (candidate_column + 0.5) * info.resolution
                center_y = (candidate_row + 0.5) * info.resolution
                distance = math.hypot(center_x - x_local, center_y - y_local)
                if distance <= snap_radius and distance < best_distance:
                    best_cell = candidate_row * info.width + candidate_column
                    best_distance = distance
        if best_cell is not None:
            key = (best_cell, int(label))
            seed_votes[key] = seed_votes.get(key, 0.0) + float(weight)

    if not seed_votes:
        return empty

    fill_cells = max(0, math.ceil(fill_radius / info.resolution))
    cell_votes: dict[tuple[int, int], float] = {}
    for (cell, label), weight in seed_votes.items():
        row, column = divmod(cell, info.width)
        for row_offset in range(-fill_cells, fill_cells + 1):
            candidate_row = row + row_offset
            if candidate_row < 0 or candidate_row >= info.height:
                continue
            for column_offset in range(-fill_cells, fill_cells + 1):
                candidate_column = column + column_offset
                if candidate_column < 0 or candidate_column >= info.width:
                    continue
                distance = math.hypot(row_offset, column_offset)
                if distance * info.resolution > fill_radius:
                    continue
                if grid[candidate_row, candidate_column] < occupancy_threshold:
                    continue
                candidate = candidate_row * info.width + candidate_column
                key = (candidate, label)
                cell_votes[key] = cell_votes.get(key, 0.0) + weight / (
                    1.0 + distance
                )

    votes_by_cell: dict[int, dict[int, float]] = {}
    for (cell, label), weight in cell_votes.items():
        votes_by_cell.setdefault(cell, {})[label] = weight

    output_points = []
    output_labels = []
    output_totals = []
    output_ratios = []
    output_cells = []
    for cell in sorted(votes_by_cell):
        class_votes = votes_by_cell[cell]
        label = max(class_votes, key=class_votes.get)
        total = sum(class_votes.values())
        row, column = divmod(cell, info.width)
        x_local = (column + 0.5) * info.resolution
        y_local = (row + 0.5) * info.resolution
        output_points.append((
            origin.position.x + cosine * x_local - sine * y_local,
            origin.position.y + sine * x_local + cosine * y_local,
            layer_z,
        ))
        output_labels.append(label)
        output_totals.append(max(1, min(round(total), 65535)))
        output_ratios.append(class_votes[label] / total)
        output_cells.append((row, column))

    return (
        np.asarray(output_points, dtype=np.float32),
        np.asarray(output_labels, dtype=np.uint16),
        np.asarray(output_totals, dtype=np.uint16),
        np.asarray(output_ratios, dtype=np.float32),
        np.asarray(output_cells, dtype=np.int32),
    )


class LocalMockSegmenter:
    """Credential-free semantic provider used by the public launch."""

    @property
    def provider(self) -> str:
        return "local-mock"

    def predict(self, image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        height, width = image.shape[:2]
        if image.ndim < 2 or height < 1 or width < 1:
            raise ValueError("image must have non-empty height and width")
        # Background-only output intentionally produces no semantic obstacles.
        # RGB/depth transport and map publication remain available for adapters.
        labels = np.zeros((height, width), dtype=np.uint8)
        confidence = np.zeros((height, width), dtype=np.float32)
        return labels, confidence


class SemanticMapper(Node):
    """Fuse provider-neutral semantic predictions into a map-frame cloud."""

    def __init__(self) -> None:
        super().__init__("semantic_mapper")
        self.declare_parameter("rgb_topic", "/isaac/color_image_raw")
        self.declare_parameter(
            "depth_topic", "/isaac/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/isaac/color/camera_info")
        self.declare_parameter("occupancy_map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("max_inference_rate", 5.0)
        self.declare_parameter("voxel_size", 0.08)
        self.declare_parameter("pixel_stride", 4)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("depth_min", 0.30)
        self.declare_parameter("depth_max", 6.0)
        self.declare_parameter("pixel_confidence_min", 0.35)
        self.declare_parameter("min_voxel_votes", 3)
        self.declare_parameter("min_vote_ratio", 0.60)
        self.declare_parameter("publish_every_frames", 5)
        self.declare_parameter("self_mask_polygon", "")
        self.declare_parameter("ignored_class_ids", "0,3,5")
        self.declare_parameter("occupancy_threshold", 50)
        self.declare_parameter("obstacle_snap_radius", 0.15)
        self.declare_parameter("obstacle_fill_radius", 0.15)
        self.declare_parameter("obstacle_layer_z", 0.06)
        self.declare_parameter("label_min_cells", 20)

        self._segmenter = LocalMockSegmenter()
        self._map = VoxelVoteMap(
            float(self.get_parameter("voxel_size").value),
            len(MOCK_SEMANTIC_CLASSES),
        )
        self._bridge = CvBridge()
        self._palette = make_palette(len(MOCK_SEMANTIC_CLASSES))
        self._camera_info: Optional[CameraInfo] = None
        self._occupancy_map: Optional[OccupancyGrid] = None
        self._active = True
        self._frame_count = 0
        self._last_inference = 0.0
        self._inference_period = 1.0 / float(
            self.get_parameter("max_inference_rate").value
        )
        self._pixel_stride = int(self.get_parameter("pixel_stride").value)
        self._depth_scale = float(self.get_parameter("depth_scale").value)
        self._depth_min = float(self.get_parameter("depth_min").value)
        self._depth_max = float(self.get_parameter("depth_max").value)
        self._pixel_confidence_min = float(
            self.get_parameter("pixel_confidence_min").value
        )
        self._min_voxel_votes = int(
            self.get_parameter("min_voxel_votes").value
        )
        self._min_vote_ratio = float(
            self.get_parameter("min_vote_ratio").value
        )
        self._publish_every_frames = int(
            self.get_parameter("publish_every_frames").value
        )
        mask_text = str(self.get_parameter("self_mask_polygon").value)
        self._self_mask_polygon = [
            float(value) for value in mask_text.split(",") if value.strip()
        ]
        ignored_text = str(self.get_parameter("ignored_class_ids").value)
        self._ignored_class_ids = {
            int(value) for value in ignored_text.split(",") if value.strip()
        }
        self._occupancy_threshold = int(
            self.get_parameter("occupancy_threshold").value
        )
        self._obstacle_snap_radius = float(
            self.get_parameter("obstacle_snap_radius").value
        )
        self._obstacle_fill_radius = float(
            self.get_parameter("obstacle_fill_radius").value
        )
        self._obstacle_layer_z = float(
            self.get_parameter("obstacle_layer_z").value
        )
        self._label_min_cells = int(
            self.get_parameter("label_min_cells").value
        )
        if (
            self._inference_period <= 0.0
            or self._pixel_stride < 1
            or self._publish_every_frames < 1
            or not 0.0 <= self._min_vote_ratio <= 1.0
            or self._obstacle_snap_radius < 0.0
            or self._obstacle_fill_radius < 0.0
            or self._label_min_cells < 1
        ):
            raise ValueError(
                "invalid semantic mapping rate, stride, or vote parameters"
            )

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        image_output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2, "/semantic_mapping/cloud", map_qos
        )
        self._obstacle_cloud_pub = self.create_publisher(
            PointCloud2, "/semantic_mapping/obstacle_cloud", map_qos
        )
        self._label_pub = self.create_publisher(
            MarkerArray, "/semantic_mapping/labels", map_qos
        )
        self._overlay_pub = self.create_publisher(
            Image, "/semantic_mapping/overlay", image_output_qos
        )
        self._class_id_pub = self.create_publisher(
            Image, "/semantic_mapping/class_ids", image_output_qos
        )
        self._class_confidence_pub = self.create_publisher(
            Image, "/semantic_mapping/class_confidence", image_output_qos
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("occupancy_map_topic").value),
            self._on_occupancy_map,
            map_qos,
        )
        self.create_subscription(
            Bool, "/semantic_mapping/active", self._on_mapping_active, map_qos
        )
        self.create_subscription(
            PointStamped, "/clicked_point", self._on_goal_fallback, 10
        )
        self._rgb_sub = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("rgb_topic").value),
            qos_profile=sensor_qos,
        )
        self._depth_sub = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("depth_topic").value),
            qos_profile=sensor_qos,
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub], queue_size=4, slop=0.08
        )
        self._sync.registerCallback(self._on_images)
        self.get_logger().info(
            "Semantic mapping active: local mock provider, "
            f"voxel={self._map.voxel_size:.3f}m"
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self._camera_info = message

    def _on_occupancy_map(self, message: OccupancyGrid) -> None:
        self._occupancy_map = message
        if self._frame_count:
            self._publish_obstacle_layer()

    def _on_mapping_active(self, message: Bool) -> None:
        if self._active and not message.data:
            self._freeze()

    def _on_goal_fallback(self, _: PointStamped) -> None:
        if self._active:
            self._freeze()

    def _freeze(self) -> None:
        self._active = False
        self._publish_cloud()
        self.get_logger().info(
            f"Semantic map frozen at first goal: {len(self._map)} raw voxels"
        )

    def _on_images(self, rgb_message: Image, depth_message: Image) -> None:
        if not self._active or self._camera_info is None:
            return
        now = time.monotonic()
        if now - self._last_inference < self._inference_period:
            return
        self._last_inference = now
        try:
            image = self._bridge.imgmsg_to_cv2(
                rgb_message, desired_encoding="rgb8"
            )
            depth = self._bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="passthrough"
            )
            depth = np.asarray(depth, dtype=np.float32)
            if depth_message.encoding in ("16UC1", "mono16"):
                depth *= self._depth_scale
            if depth.shape != image.shape[:2]:
                depth = cv2.resize(
                    depth,
                    (image.shape[1], image.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            labels, confidence = self._segmenter.predict(image)
            self_mask = make_polygon_mask(
                image.shape[0], image.shape[1], self._self_mask_polygon
            )
            self._publish_class_images(
                rgb_message.header, labels, confidence
            )
            self._publish_overlay(rgb_message.header, image, labels, self_mask)
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                rgb_message.header.frame_id,
                rclpy.time.Time.from_msg(rgb_message.header.stamp),
                timeout=Duration(seconds=0.1),
            )
            points, sampled_labels = self._project(
                depth, labels, confidence, self_mask
            )
            points = self._transform(points, transform)
            added = self._map.add_frame(points, sampled_labels)
            self._frame_count += 1
            if self._frame_count % self._publish_every_frames == 0:
                self._publish_cloud()
            self.get_logger().debug(
                f"Semantic frame {self._frame_count}: {added} voxel votes",
                throttle_duration_sec=1.0,
            )
        except (cv2.error, ValueError, tf2_ros.TransformException) as error:
            self.get_logger().warning(
                f"Skipping semantic frame: {error}", throttle_duration_sec=1.0
            )

    def _project(
        self,
        depth: np.ndarray,
        labels: np.ndarray,
        confidence: np.ndarray,
        self_mask: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        stride = self._pixel_stride
        sampled_depth = depth[::stride, ::stride]
        sampled_labels = labels[::stride, ::stride]
        sampled_confidence = confidence[::stride, ::stride]
        sampled_mask = self_mask[::stride, ::stride]
        rows, columns = np.meshgrid(
            np.arange(0, depth.shape[0], stride),
            np.arange(0, depth.shape[1], stride),
            indexing="ij",
        )
        valid = (
            np.isfinite(sampled_depth)
            & (sampled_depth >= self._depth_min)
            & (sampled_depth <= self._depth_max)
            & (sampled_confidence >= self._pixel_confidence_min)
            & ~sampled_mask
        )
        z = sampled_depth[valid]
        camera = self._camera_info
        x = (columns[valid] - camera.k[2]) * z / camera.k[0]
        y = (rows[valid] - camera.k[5]) * z / camera.k[4]
        return np.column_stack((x, y, z)), sampled_labels[valid]

    @staticmethod
    def _transform(points: np.ndarray, transform) -> np.ndarray:
        if not len(points):
            return points.reshape(-1, 3)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
        matrix = np.array(
            [
                [
                    1 - 2 * (y * y + z * z),
                    2 * (x * y - z * w),
                    2 * (x * z + y * w),
                ],
                [
                    2 * (x * y + z * w),
                    1 - 2 * (x * x + z * z),
                    2 * (y * z - x * w),
                ],
                [
                    2 * (x * z - y * w),
                    2 * (y * z + x * w),
                    1 - 2 * (x * x + y * y),
                ],
            ],
            dtype=np.float64,
        )
        offset = np.array(
            [translation.x, translation.y, translation.z], dtype=np.float64
        )
        return points @ matrix.T + offset

    def _publish_overlay(
        self,
        header: Header,
        image: np.ndarray,
        labels: np.ndarray,
        self_mask: np.ndarray,
    ) -> None:
        semantic = self._palette[labels]
        overlay = cv2.addWeighted(image, 0.45, semantic, 0.55, 0.0)
        overlay[self_mask] = (20, 20, 20)
        message = self._bridge.cv2_to_imgmsg(overlay, encoding="rgb8")
        message.header = header
        self._overlay_pub.publish(message)

    def _publish_class_images(
        self,
        header: Header,
        labels: np.ndarray,
        confidence: np.ndarray,
    ) -> None:
        class_message = self._bridge.cv2_to_imgmsg(
            np.asarray(labels, dtype=np.uint8), encoding="mono8"
        )
        class_message.header = header
        self._class_id_pub.publish(class_message)
        confidence_message = self._bridge.cv2_to_imgmsg(
            np.asarray(confidence, dtype=np.float32), encoding="32FC1"
        )
        confidence_message.header = header
        self._class_confidence_pub.publish(confidence_message)

    def _publish_cloud(self) -> None:
        points, labels, totals, ratios = self._map.resolved(
            self._min_voxel_votes, self._min_vote_ratio
        )
        self._cloud_pub.publish(
            self._make_cloud_message(points, labels, totals, ratios)
        )
        self._publish_obstacle_layer((points, labels, totals, ratios))

    def _publish_obstacle_layer(self, resolved=None) -> None:
        if self._occupancy_map is None:
            return
        if resolved is None:
            resolved = self._map.resolved(
                self._min_voxel_votes, self._min_vote_ratio
            )
        points, labels, totals, ratios, cells = build_obstacle_layer(
            *resolved,
            self._occupancy_map,
            self._ignored_class_ids,
            self._occupancy_threshold,
            self._obstacle_snap_radius,
            self._obstacle_fill_radius,
            self._obstacle_layer_z,
        )
        self._obstacle_cloud_pub.publish(
            self._make_cloud_message(points, labels, totals, ratios)
        )
        self._publish_labels(points, labels, totals, ratios, cells)

    def _publish_labels(
        self,
        points: np.ndarray,
        labels: np.ndarray,
        totals: np.ndarray,
        ratios: np.ndarray,
        cells: np.ndarray,
    ) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = str(self.get_parameter("map_frame").value)
        delete_all = Marker()
        delete_all.header = header
        delete_all.action = Marker.DELETEALL
        marker_array = MarkerArray(markers=[delete_all])
        if not len(points):
            self._label_pub.publish(marker_array)
            return

        occupancy = self._occupancy_map.info
        obstacle_mask = np.zeros(
            (occupancy.height, occupancy.width), dtype=np.uint8
        )
        obstacle_mask[cells[:, 0], cells[:, 1]] = 1
        component_count, components = cv2.connectedComponents(
            obstacle_mask, connectivity=8
        )
        marker_id = 0
        for component in range(1, component_count):
            members = (
                components[cells[:, 0], cells[:, 1]] == component
            )
            if int(np.count_nonzero(members)) < self._label_min_cells:
                continue
            weights = totals[members].astype(np.float64) * ratios[members]
            class_votes = np.bincount(
                labels[members],
                weights=weights,
                minlength=len(MOCK_SEMANTIC_CLASSES),
            )
            label = int(np.argmax(class_votes))
            center = points[members].mean(axis=0)
            marker = Marker()
            marker.header = header
            marker.ns = "semantic_obstacle_labels"
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = self._obstacle_layer_z + 0.12
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.25
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = MOCK_SEMANTIC_CLASSES[label]
            marker_array.markers.append(marker)
            marker_id += 1
        self._label_pub.publish(marker_array)

    def _make_cloud_message(
        self,
        points: np.ndarray,
        labels: np.ndarray,
        totals: np.ndarray,
        ratios: np.ndarray,
    ) -> PointCloud2:
        data = np.zeros(
            len(points),
            dtype=np.dtype(
                [
                    ("x", "<f4"),
                    ("y", "<f4"),
                    ("z", "<f4"),
                    ("rgb", "<u4"),
                    ("semantic_id", "<u2"),
                    ("vote_count", "<u2"),
                    ("confidence", "<f4"),
                ]
            ),
        )
        if len(points):
            data["x"], data["y"], data["z"] = points.T
            colors = self._palette[labels]
            data["rgb"] = (
                colors[:, 0].astype(np.uint32) << 16
                | colors[:, 1].astype(np.uint32) << 8
                | colors[:, 2].astype(np.uint32)
            )
            data["semantic_id"] = labels
            data["vote_count"] = totals
            data["confidence"] = ratios
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("map_frame").value)
        message.height = 1
        message.width = len(data)
        message.fields = [
            PointField(
                name="x", offset=0, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="y", offset=4, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="z", offset=8, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="rgb", offset=12, datatype=PointField.UINT32, count=1
            ),
            PointField(
                name="semantic_id",
                offset=16,
                datatype=PointField.UINT16,
                count=1,
            ),
            PointField(
                name="vote_count",
                offset=18,
                datatype=PointField.UINT16,
                count=1,
            ),
            PointField(
                name="confidence",
                offset=20,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        message.is_bigendian = False
        message.point_step = data.dtype.itemsize
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = data.tobytes()
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapper()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
