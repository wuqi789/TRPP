"""ROS-independent geometry, synchronization, clustering, and safety policy."""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class CorridorObstacle:
    point: tuple[float, float]
    along_path: float
    lateral: float
    source_index: int


@dataclass(frozen=True)
class ApproachControl:
    linear: float
    angular: float
    cross_track_error: float
    heading_error: float
    reached: bool


@dataclass(frozen=True)
class TraversalControl:
    linear: float
    angular: float
    cross_track_error: float
    heading_error: float
    stopped: bool


@dataclass(frozen=True)
class TargetBandObservation:
    """Request-locked target and non-target scan partition."""

    target_indices: tuple[int, ...]
    external_indices: tuple[int, ...]
    clearance: float
    bearing: float


def median_clearance(samples: Sequence[float], window: int = 3) -> float | None:
    """Return a robust clearance estimate from the newest finite samples."""
    if window <= 0:
        raise ValueError("window must be positive")
    values = [float(value) for value in samples if math.isfinite(float(value))]
    if not values:
        return None
    return float(statistics.median(values[-window:]))


def rear_sector_observable(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    *,
    half_angle: float = 0.60,
    minimum_coverage: float = 0.80,
    negative_one_is_no_return: bool = False,
) -> tuple[bool, float, int]:
    """Report whether a rear LaserScan sector is observed.

    Positive infinity is a valid no-return observation. Isaac's RTX LaserScan
    writer uses finite ``-1.0`` for the same condition; callers must explicitly
    enable that simulation-only convention. NaN, negative infinity, zero, and
    all other out-of-range finite values are not observations.
    """
    if angle_increment <= 0.0 or range_min < 0.0 or range_max <= range_min:
        raise ValueError("invalid LaserScan geometry")
    if half_angle <= 0.0 or not 0.0 < minimum_coverage <= 1.0:
        raise ValueError("invalid rear-sector parameters")
    selected = observed = 0
    for index, raw_value in enumerate(ranges):
        angle = normalize_angle(float(angle_min) + index * float(angle_increment) - math.pi)
        if abs(angle) > half_angle:
            continue
        selected += 1
        value = float(raw_value)
        if math.isinf(value) and value > 0.0:
            observed += 1
        elif negative_one_is_no_return and math.isclose(
            value, -1.0, rel_tol=0.0, abs_tol=1e-6
        ):
            observed += 1
        elif math.isfinite(value) and range_min <= value <= range_max:
            observed += 1
    coverage = float(observed) / float(selected) if selected else 0.0
    return selected > 0 and coverage >= minimum_coverage, coverage, selected


def traversal_sensor_inputs_ready(
    received: dict[str, float], now: float, freshness: float
) -> bool:
    """Return startup sensor readiness without requiring a navigation path."""
    if freshness <= 0.0:
        return False
    return all(
        now - float(received.get(name, float("-inf"))) <= freshness
        for name in ("scan", "image", "camera")
    )


def rejected_cache_active(
    robot: tuple[float, float],
    recovery_anchor: tuple[float, float],
    obstacle_reference: tuple[float, float],
    *,
    motion_limit: float,
    obstacle_neighborhood: float,
) -> bool:
    """Keep a fail-closed decision active while recovering or bypassing it."""
    if motion_limit <= 0.0 or obstacle_neighborhood <= 0.0:
        raise ValueError("cache limits must be positive")
    return (
        math.dist(robot, recovery_anchor) <= motion_limit
        or math.dist(robot, obstacle_reference) <= obstacle_neighborhood
    )


def filter_track_loss_in_completion_grace(
    pass_progress: float, pass_distance: float, grace_distance: float
) -> bool:
    """Allow a target to leave rear LiDAR view just before pass completion."""
    if pass_distance <= 0.0 or grace_distance < 0.0:
        raise ValueError("filter pass distances are invalid")
    return float(pass_progress) >= max(
        0.0, float(pass_distance) - float(grace_distance)
    )


def closest_path_obstacle(
    points: Sequence[tuple[float, float]],
    path: Sequence[tuple[float, float]],
    robot: tuple[float, float],
    *,
    trigger_distance: float = 1.50,
    half_width: float = 0.433,
) -> CorridorObstacle | None:
    """Select the nearest scan point lying in the forward path corridor."""
    if len(path) < 2 or trigger_distance <= 0.0 or half_width <= 0.0:
        return None
    segments: list[tuple[np.ndarray, np.ndarray, float, float]] = []
    cumulative = 0.0
    for first, second in zip(path, path[1:]):
        a, b = np.asarray(first, float), np.asarray(second, float)
        length = float(np.linalg.norm(b - a))
        if length > 1e-6:
            segments.append((a, b, length, cumulative))
            cumulative += length
    if not segments:
        return None
    robot_array = np.asarray(robot, float)
    robot_along, _, _, robot_direction = _project_polyline(robot_array, segments)
    best = None
    for index, value in enumerate(points):
        point = np.asarray(value, float)
        if not np.all(np.isfinite(point)):
            continue
        along, lateral, projected, _ = _project_polyline(point, segments)
        forward = along - robot_along
        # A point just behind the first path endpoint projects onto that endpoint and
        # would otherwise have ``forward == 0``.  Reject it in the local forward
        # half-plane as well as by polyline arc length.
        local_forward = float(np.dot(point - robot_array, robot_direction))
        if (
            forward < 0.0
            or local_forward <= 1e-6
            or forward > trigger_distance
            or lateral > half_width
        ):
            continue
        if float(np.dot(projected - robot_array, projected - robot_array)) > (
            trigger_distance + half_width
        ) ** 2:
            continue
        candidate = CorridorObstacle((float(point[0]), float(point[1])), forward, lateral, index)
        if best is None or (candidate.along_path, candidate.lateral) < (
            best.along_path, best.lateral
        ):
            best = candidate
    return best


def _project_polyline(point, segments):
    best = None
    for a, b, length, cumulative in segments:
        direction = (b - a) / length
        local = float(np.clip(np.dot(point - a, direction), 0.0, length))
        projected = a + direction * local
        lateral = float(np.linalg.norm(point - projected))
        value = (cumulative + local, lateral, projected, direction)
        if best is None or lateral < best[1]:
            best = value
    return best


def path_corridor_indices(
    points_map: np.ndarray,
    path: Sequence[tuple[float, float]],
    obstacle_center: tuple[float, float],
    *,
    half_width: float,
    before_obstacle: float,
    after_obstacle: float,
) -> list[int]:
    """Return every scan point inside an authorized path-aligned corridor.

    Unlike :func:`scan_cluster_indices`, this selection deliberately ignores
    scan continuity and range jumps.  A curtain can appear as several disjoint
    LaserScan components, so all components occupying the verified local path
    band must be removed together.  Points outside the band remain available
    to the local planner for wall and unrelated-obstacle avoidance.
    """
    points = np.asarray(points_map, dtype=float)
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError("points_map must be an Nx2 or Nx3 array")
    if half_width <= 0.0 or before_obstacle < 0.0 or after_obstacle < 0.0:
        raise ValueError("corridor dimensions must be positive or zero")
    if len(path) < 2:
        return []

    segments: list[tuple[np.ndarray, np.ndarray, float, float]] = []
    cumulative = 0.0
    for first, second in zip(path, path[1:]):
        a, b = np.asarray(first, float), np.asarray(second, float)
        length = float(np.linalg.norm(b - a))
        if length > 1e-6:
            segments.append((a, b, length, cumulative))
            cumulative += length
    if not segments:
        return []

    obstacle = np.asarray(obstacle_center, dtype=float)
    if obstacle.shape != (2,) or not np.all(np.isfinite(obstacle)):
        return []
    projected_obstacle = _project_polyline(obstacle, segments)
    if projected_obstacle is None:
        return []
    obstacle_along = projected_obstacle[0]
    lower = obstacle_along - float(before_obstacle)
    upper = obstacle_along + float(after_obstacle)

    selected = []
    for index, point in enumerate(points[:, :2]):
        if not np.all(np.isfinite(point)):
            continue
        projected = _project_polyline(point, segments)
        if projected is None:
            continue
        along, lateral, _, _ = projected
        if lower <= along <= upper and lateral <= float(half_width):
            selected.append(index)
    return selected


def target_band_observation(
    points_map: np.ndarray,
    points_robot: np.ndarray,
    path: Sequence[tuple[float, float]],
    obstacle_center: tuple[float, float],
    robot: tuple[float, float, float],
    *,
    footprint_length: float,
    footprint_width: float,
    half_width: float = 0.433,
    before_obstacle: float = 0.45,
    after_obstacle: float = 0.80,
    face_depth: float = 0.05,
) -> TargetBandObservation | None:
    """Measure all components in a request-locked path occupancy band.

    Clearance is measured along the locked path from the Scout support point in
    the path direction to the nearest target face. Points outside the complete
    band are returned separately for emergency-clearance checks.
    """
    mapped = np.asarray(points_map, dtype=float)
    local = np.asarray(points_robot, dtype=float)
    if (
        mapped.ndim != 2
        or local.ndim != 2
        or mapped.shape[0] != local.shape[0]
        or mapped.shape[1] < 2
        or local.shape[1] < 2
    ):
        raise ValueError("points_map and points_robot must be aligned Nx2/Nx3 arrays")
    if footprint_length <= 0.0 or footprint_width <= 0.0 or face_depth < 0.0:
        raise ValueError("invalid footprint or target-face dimensions")

    target = path_corridor_indices(
        mapped,
        path,
        obstacle_center,
        half_width=half_width,
        before_obstacle=before_obstacle,
        after_obstacle=after_obstacle,
    )
    target_set = set(target)
    external = tuple(
        index
        for index, point in enumerate(mapped[:, :2])
        if index not in target_set and np.all(np.isfinite(point))
    )
    if not target:
        return None

    segments: list[tuple[np.ndarray, np.ndarray, float, float]] = []
    cumulative = 0.0
    for first, second in zip(path, path[1:]):
        a, b = np.asarray(first, float), np.asarray(second, float)
        length = float(np.linalg.norm(b - a))
        if length > 1e-6:
            segments.append((a, b, length, cumulative))
            cumulative += length
    if not segments:
        return None

    robot_position = np.asarray(robot[:2], dtype=float)
    projected_robot = _project_polyline(robot_position, segments)
    if projected_robot is None:
        return None
    robot_along, _, _, path_direction = projected_robot
    heading = np.asarray(
        [math.cos(float(robot[2])), math.sin(float(robot[2]))], dtype=float
    )
    side = np.asarray([-heading[1], heading[0]], dtype=float)
    front_extent = (
        footprint_length / 2.0 * abs(float(np.dot(heading, path_direction)))
        + footprint_width / 2.0 * abs(float(np.dot(side, path_direction)))
    )

    along_by_index: dict[int, float] = {}
    for index in target:
        projected = _project_polyline(mapped[index, :2], segments)
        if projected is not None and projected[0] >= robot_along - 1e-6:
            along_by_index[index] = float(projected[0])
    if not along_by_index:
        return None
    nearest_along = min(along_by_index.values())
    face = [
        index for index, along in along_by_index.items()
        if along <= nearest_along + float(face_depth) + 1e-9
    ]
    face_x = float(np.median(local[face, 0]))
    face_y = float(np.median(local[face, 1]))
    bearing = math.atan2(face_y, face_x)
    clearance = nearest_along - robot_along - front_extent
    return TargetBandObservation(
        tuple(target), external, float(clearance), float(bearing)
    )


def synchronized(stamps: Iterable[float], tolerance: float = 0.20) -> bool:
    values = tuple(float(value) for value in stamps)
    return bool(values) and all(math.isfinite(value) for value in values) and (
        max(values) - min(values) <= tolerance + 1e-6
    )


def outside_footprint_mask(
    points_robot: np.ndarray,
    footprint_length: float,
    footprint_width: float,
    *,
    padding: float = 0.0,
) -> np.ndarray:
    """Return a mask which removes LiDAR returns from inside the robot body."""
    points = np.asarray(points_robot, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_robot must be an Nx3 array")
    if footprint_length <= 0.0 or footprint_width <= 0.0 or padding < 0.0:
        raise ValueError("footprint dimensions must be positive and padding non-negative")
    half_length = footprint_length / 2.0 + padding
    half_width = footprint_width / 2.0 + padding
    finite = np.isfinite(points).all(axis=1)
    inside = (np.abs(points[:, 0]) <= half_length) & (
        np.abs(points[:, 1]) <= half_width
    )
    return finite & ~inside


def nearest_tracked_point(
    points_map: np.ndarray,
    reference: tuple[float, float],
    maximum_distance: float,
) -> int | None:
    """Return the nearest scan point to a fixed obstacle track in map space."""
    points = np.asarray(points_map, dtype=float)
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError("points_map must be an Nx2 or Nx3 array")
    if maximum_distance <= 0.0:
        raise ValueError("maximum_distance must be positive")
    if not len(points):
        return None
    delta = points[:, :2] - np.asarray(reference, dtype=float)
    distances = np.linalg.norm(delta, axis=1)
    index = int(np.argmin(distances))
    return index if float(distances[index]) <= maximum_distance else None


def minimum_external_scan_clearance(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    *,
    lidar_offset_x: float,
    lidar_offset_y: float,
    footprint_length: float,
    footprint_width: float,
) -> float | None:
    """Return the nearest external LaserScan point's clearance from the body.

    Scan points are transformed from the LiDAR origin into the axis-aligned
    robot footprint. Returns inside the footprint are self echoes and do not
    contribute to obstacle clearance.
    """
    if (
        footprint_length <= 0.0
        or footprint_width <= 0.0
        or angle_increment <= 0.0
        or range_min < 0.0
        or range_max <= range_min
    ):
        raise ValueError("invalid scan or footprint geometry")
    half_length = footprint_length / 2.0
    half_width = footprint_width / 2.0
    minimum = None
    for index, raw_distance in enumerate(ranges):
        distance = float(raw_distance)
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        angle = float(angle_min) + index * float(angle_increment)
        x = float(lidar_offset_x) + distance * math.cos(angle)
        y = float(lidar_offset_y) + distance * math.sin(angle)
        dx = max(abs(x) - half_length, 0.0)
        dy = max(abs(y) - half_width, 0.0)
        if dx == 0.0 and dy == 0.0:
            continue
        clearance = math.hypot(dx, dy)
        minimum = clearance if minimum is None else min(minimum, clearance)
    return minimum


def point_clearance_from_footprint(
    points_robot: np.ndarray,
    footprint_length: float,
    footprint_width: float,
) -> float | None:
    """Return the nearest point-to-body clearance for a selected obstacle."""
    points = np.asarray(points_robot, dtype=float)
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError("points_robot must be Nx2 or Nx3")
    if footprint_length <= 0.0 or footprint_width <= 0.0:
        raise ValueError("footprint dimensions must be positive")
    half_length = footprint_length / 2.0
    half_width = footprint_width / 2.0
    values = []
    for x, y in points[:, :2]:
        if not math.isfinite(float(x)) or not math.isfinite(float(y)):
            continue
        dx = max(abs(float(x)) - half_length, 0.0)
        dy = max(abs(float(y)) - half_width, 0.0)
        if dx == 0.0 and dy == 0.0:
            continue
        values.append(math.hypot(dx, dy))
    return min(values) if values else None


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def path_tangent_heading_error(
    robot: tuple[float, float, float],
    path: Sequence[tuple[float, float]],
) -> float | None:
    """Return robot heading error to the closest non-degenerate path segment."""
    if len(path) < 2:
        return None
    segments = []
    cumulative = 0.0
    for first, second in zip(path, path[1:]):
        a, b = np.asarray(first, float), np.asarray(second, float)
        length = float(np.linalg.norm(b - a))
        if length > 1e-6:
            segments.append((a, b, length, cumulative))
            cumulative += length
    if not segments:
        return None
    _, _, _, direction = _project_polyline(
        np.asarray(robot[:2], dtype=float), segments
    )
    path_heading = math.atan2(float(direction[1]), float(direction[0]))
    return normalize_angle(path_heading - float(robot[2]))


def approach_path_command(
    robot: tuple[float, float, float],
    path: Sequence[tuple[float, float]],
    clearance: float,
    *,
    target_clearance: float = 0.20,
    clearance_tolerance: float = 0.02,
    lookahead: float = 0.35,
    far_speed: float = 0.15,
    near_speed: float = 0.08,
    near_distance: float = 0.50,
    max_angular: float = 0.25,
) -> ApproachControl:
    """Follow a locked path while closing on a tracked obstacle."""
    if len(path) < 2 or lookahead <= 0.0:
        raise ValueError("a non-degenerate path and positive lookahead are required")
    if target_clearance <= 0.0 or clearance_tolerance <= 0.0:
        raise ValueError("clearance parameters must be positive")
    if clearance <= target_clearance + clearance_tolerance:
        return ApproachControl(0.0, 0.0, 0.0, 0.0, True)

    position = np.asarray(robot[:2], dtype=float)
    segments = []
    cumulative = 0.0
    for first, second in zip(path, path[1:]):
        a, b = np.asarray(first, float), np.asarray(second, float)
        length = float(np.linalg.norm(b - a))
        if length > 1e-6:
            segments.append((a, b, length, cumulative))
            cumulative += length
    if not segments:
        raise ValueError("path has no non-degenerate segment")
    along, lateral, projected, _ = _project_polyline(position, segments)
    target_along = min(cumulative, along + lookahead)
    target = segments[-1][1]
    for a, b, length, start in segments:
        if target_along <= start + length:
            target = a + (b - a) * ((target_along - start) / length)
            break
    heading = math.atan2(float(target[1] - position[1]), float(target[0] - position[0]))
    heading_error = normalize_angle(heading - float(robot[2]))
    direction = np.asarray([math.cos(robot[2]), math.sin(robot[2])])
    offset = projected - position
    cross = float(direction[0] * offset[1] - direction[1] * offset[0])
    cap = far_speed if clearance > near_distance else near_speed
    speed = min(cap, max(0.0, 0.8 * (clearance - target_clearance)))
    speed *= max(0.0, math.cos(heading_error))
    angular = max(-max_angular, min(max_angular, 1.5 * heading_error - 0.8 * cross))
    return ApproachControl(speed, angular, cross, heading_error, False)


def traversal_path_command(
    robot: tuple[float, float, float],
    path: Sequence[tuple[float, float]],
    requested_linear: float,
    *,
    maximum_linear: float = 0.15,
    lookahead: float = 0.35,
    maximum_angular: float = 0.25,
    maximum_cross_track_error: float = 0.15,
    maximum_heading_error: float = 1.20,
) -> TraversalControl:
    """Keep an authorized crossing on its request-locked path.

    NeuPAN retains stop authority through ``requested_linear``. Its angular
    command is deliberately not reused because NeuPAN may have replanned an
    avoidance curve while the traversal gate held the stationary robot.
    """
    if maximum_linear <= 0.0 or maximum_angular <= 0.0:
        raise ValueError("traversal speed limits must be positive")
    command = approach_path_command(
        robot,
        path,
        math.inf,
        lookahead=lookahead,
        far_speed=maximum_linear,
        near_speed=maximum_linear,
        max_angular=maximum_angular,
    )
    unsafe_deviation = (
        abs(command.cross_track_error) > maximum_cross_track_error
        or abs(command.heading_error) > maximum_heading_error
    )
    requested = float(requested_linear)
    if not math.isfinite(requested) or requested <= 0.0 or unsafe_deviation:
        return TraversalControl(
            0.0,
            0.0,
            command.cross_track_error,
            command.heading_error,
            unsafe_deviation,
        )
    return TraversalControl(
        min(maximum_linear, requested, command.linear),
        command.angular,
        command.cross_track_error,
        command.heading_error,
        False,
    )


def movable_probability(displacement: float) -> float:
    """Map verified rigid-object displacement to the agreed mechanical score."""
    value = float(displacement)
    if not math.isfinite(value) or value < 0.02:
        return 0.0
    return min(1.0, 0.60 + 0.40 * (value - 0.02) / 0.03)


def fusion_probability(dino: float, llm: float, mechanical: float) -> float:
    values = [max(0.0, min(1.0, float(x))) for x in (dino, llm, mechanical)]
    return 0.25 * values[0] + 0.25 * values[1] + 0.50 * values[2]


def three_modal_authorized(
    dino: float,
    llm: float,
    mechanical: float,
    *,
    mechanical_positive: bool,
    mechanical_action_succeeded: bool,
    arm_returned: bool,
    threshold: float = 0.75,
) -> bool:
    return (
        bool(mechanical_positive)
        and bool(mechanical_action_succeeded)
        and bool(arm_returned)
        and fusion_probability(dino, llm, mechanical) >= float(threshold)
    )


def project_roi(
    points_scan: np.ndarray,
    scan_to_camera: np.ndarray,
    camera_matrix: np.ndarray,
    image_width: int,
    image_height: int,
    *,
    padding: int = 12,
) -> tuple[int, int, int, int] | None:
    """Project Nx3 scan points into camera pixels and return clipped xywh ROI."""
    points = np.asarray(points_scan, dtype=float)
    transform = np.asarray(scan_to_camera, dtype=float)
    intrinsics = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    if points.ndim != 2 or points.shape[1] != 3 or transform.shape != (4, 4):
        raise ValueError("invalid point or transform shape")
    homogeneous = np.column_stack((points, np.ones(len(points))))
    camera = (transform @ homogeneous.T).T[:, :3]
    camera = camera[camera[:, 2] > 1e-4]
    if not len(camera):
        return None
    pixels = (intrinsics @ camera.T).T
    pixels = pixels[:, :2] / pixels[:, 2:3]
    valid = (
        np.isfinite(pixels).all(axis=1)
        & (pixels[:, 0] >= -padding)
        & (pixels[:, 0] < image_width + padding)
        & (pixels[:, 1] >= -padding)
        & (pixels[:, 1] < image_height + padding)
    )
    pixels = pixels[valid]
    if not len(pixels):
        return None
    x1 = max(0, int(math.floor(float(pixels[:, 0].min()))) - padding)
    y1 = max(0, int(math.floor(float(pixels[:, 1].min()))) - padding)
    x2 = min(image_width, int(math.ceil(float(pixels[:, 0].max()))) + padding + 1)
    y2 = min(image_height, int(math.ceil(float(pixels[:, 1].max()))) + padding + 1)
    return (x1, y1, x2 - x1, y2 - y1) if x2 > x1 and y2 > y1 else None


def expand_roi(
    roi: tuple[int, int, int, int],
    image_width: int,
    image_height: int,
    *,
    minimum_width: int,
    minimum_height: int,
) -> tuple[int, int, int, int]:
    """Expand an xywh ROI around its center while staying inside the image."""
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    x, y, width, height = (int(value) for value in roi)
    if width <= 0 or height <= 0:
        raise ValueError("ROI dimensions must be positive")
    target_width = min(image_width, max(width, int(minimum_width)))
    target_height = min(image_height, max(height, int(minimum_height)))
    center_x = x + width / 2.0
    center_y = y + height / 2.0
    expanded_x = int(round(center_x - target_width / 2.0))
    expanded_y = int(round(center_y - target_height / 2.0))
    expanded_x = max(0, min(expanded_x, image_width - target_width))
    expanded_y = max(0, min(expanded_y, image_height - target_height))
    return expanded_x, expanded_y, target_width, target_height


def scan_cluster_indices(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    center_angle: float,
    center_range: float,
    *,
    range_tolerance: float = 0.12,
    angular_expansion: float = 0.08,
    edge_points: int = 3,
    seed_search_sector: float = math.radians(45.0),
) -> list[int]:
    """Return the complete continuous scan cluster around an authorization.

    ``seed_search_sector`` is only an association gate used to find the scan
    return nearest the tracked obstacle center.  It never clips the resulting
    cluster.  Once the seed is associated, every point in the continuous
    range-consistent cluster is returned; there is deliberately no angular or
    scan-fraction output cap.
    """
    count = len(ranges)
    if count == 0 or angle_increment <= 0.0:
        return []
    finite = [math.isfinite(float(value)) and float(value) > 0.0 for value in ranges]
    half_sector = max(0.0, float(seed_search_sector)) / 2.0
    candidates = [
        index for index, valid in enumerate(finite)
        if valid and abs((angle_min + index * angle_increment) - center_angle) <= half_sector
    ]
    if not candidates:
        return []
    seed = min(
        candidates,
        key=lambda index: (
            abs(float(ranges[index]) - center_range),
            abs((angle_min + index * angle_increment) - center_angle),
        ),
    )
    if abs(float(ranges[seed]) - center_range) > max(0.5, 3.0 * range_tolerance):
        return []
    maximum_range_drift = max(0.25, 3.0 * range_tolerance)
    left = right = seed
    while (
        left > 0
        and finite[left - 1]
        and abs(float(ranges[left - 1]) - float(ranges[left])) <= range_tolerance
        and abs(float(ranges[left - 1]) - center_range) <= maximum_range_drift
    ):
        left -= 1
    while (
        right + 1 < count
        and finite[right + 1]
        and abs(float(ranges[right + 1]) - float(ranges[right])) <= range_tolerance
        and abs(float(ranges[right + 1]) - center_range) <= maximum_range_drift
    ):
        right += 1
    angular_points = int(math.ceil(angular_expansion / angle_increment))
    expansion = angular_points + max(0, min(3, int(edge_points)))
    # Expand only across valid returns that still belong to the tracked range
    # envelope. Blind index expansion can jump over invalid readings and fold a
    # distant wall into a nearby object's cluster, corrupting its tracked center.
    for _ in range(expansion):
        candidate = left - 1
        if (
            candidate < 0
            or not finite[candidate]
            or abs(float(ranges[candidate]) - center_range) > maximum_range_drift
        ):
            break
        left = candidate
    for _ in range(expansion):
        candidate = right + 1
        if (
            candidate >= count
            or not finite[candidate]
            or abs(float(ranges[candidate]) - center_range) > maximum_range_drift
        ):
            break
        right = candidate
    return list(range(left, right + 1))


def transform_matrix(translation, rotation) -> np.ndarray:
    """Create a homogeneous matrix from ROS translation and xyzw quaternion objects."""
    x, y, z, w = float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm <= 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    matrix[:3, 3] = [float(translation.x), float(translation.y), float(translation.z)]
    return matrix


def velocity_gate_mode(
    state: int | None,
    *,
    status_stale: bool,
    filter_active: bool,
    filter_matches: bool,
    snapshot_pending: int = 1,
    verifying: int = 2,
    authorized: int = 3,
    traversing: int = 4,
    approaching: int = 7,
    aligning: int = 8,
    arm_ready: int = 9,
    probing: int = 10,
    arm_returning: int = 11,
    fusing: int = 12,
    recovering: int = 13,
) -> str:
    """Return NORMAL, STOP, or LIMIT for a traversal/control snapshot."""
    controlled = {
        snapshot_pending, verifying, authorized, traversing, approaching, aligning,
        arm_ready, probing, arm_returning, fusing, recovering,
    }
    # Full NeuPAN commands must never pass while a scan is still being
    # mutated. This also closes the short DDS ordering window where a task
    # terminal status arrives before the matching filter revocation.
    if filter_active:
        if (
            not status_stale
            and state in (authorized, traversing)
            and filter_matches
        ):
            return "LIMIT"
        return "STOP"
    if status_stale:
        return "STOP" if state in controlled else "NORMAL"
    if state is None:
        return "NORMAL"
    if state in (snapshot_pending, verifying):
        return "STOP"
    if state in (approaching, aligning, recovering):
        return "APPROACH"
    if state in (arm_ready, probing, arm_returning, fusing):
        return "STOP"
    if state in (authorized, traversing):
        return "STOP"
    return "NORMAL"
