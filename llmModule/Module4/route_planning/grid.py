"""Occupancy-grid conversion, rendering, connectivity, and waypoint snapping."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import io
import math
from typing import Iterable, Sequence

from PIL import Image, ImageDraw


class RouteValidationError(ValueError):
    """A waypoint cannot be represented by the configured map policy."""


@dataclass(frozen=True)
class GridPoint:
    x: float
    y: float


class GridMap:
    """Map view used to sanitize VLM points without planning robot motion."""

    def __init__(
        self,
        *,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float = 0.0,
        data: Sequence[int],
        frame_id: str = "odom",
        occupied_threshold: int = 50,
        inflation_radius: float = 0.10,
        scale: int = 4,
        avoid_regions: Iterable[tuple[float, float, float]] = (),
        pushable_regions: Iterable[tuple[float, float, float, float]] = (),
    ) -> None:
        if width <= 0 or height <= 0 or resolution <= 0.0:
            raise ValueError("grid metadata is invalid")
        if len(data) != width * height:
            raise ValueError("grid data size does not match metadata")
        if scale < 1:
            raise ValueError("render scale must be positive")
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_yaw = float(origin_yaw)
        self._origin_cos = math.cos(self.origin_yaw)
        self._origin_sin = math.sin(self.origin_yaw)
        self.frame_id = frame_id or "odom"
        self.scale = int(scale)
        self.inflation_radius = max(0.0, float(inflation_radius))
        self._base_blocked = [
            int(value) < 0 or int(value) >= occupied_threshold for value in data
        ]
        self._avoid = [False] * (self.width * self.height)
        for x, y, radius in avoid_regions:
            self._paint_disk(self._avoid, x, y, max(0.0, radius))
        self._pushable = [False] * (self.width * self.height)
        for center_x, center_y, size_x, size_y in pushable_regions:
            self._paint_box(
                self._pushable,
                center_x,
                center_y,
                max(0.0, size_x),
                max(0.0, size_y),
            )
        self._source_blocked = [
            obstacle or avoid
            for obstacle, avoid in zip(self._base_blocked, self._avoid)
        ]
        self._blocked = self._inflate(self._source_blocked, self.inflation_radius)
        self._inflated_avoid = self._inflate(self._avoid, self.inflation_radius)
        # Pushable regions may fill gaps in the occupancy map. Keep them forbidden
        # as waypoint cells while allowing unvalidated line segments to cross them.
        self._blocked = [
            blocked or pushable
            for blocked, pushable in zip(self._blocked, self._pushable)
        ]

        boundary_cells = math.ceil(self.inflation_radius / self.resolution)
        for cell_y in range(self.height):
            for cell_x in range(self.width):
                if (
                    cell_x < boundary_cells
                    or cell_y < boundary_cells
                    or cell_x >= self.width - boundary_cells
                    or cell_y >= self.height - boundary_cells
                ):
                    self._blocked[self._index(cell_x, cell_y)] = True

    @property
    def pixel_width(self) -> int:
        return self.width * self.scale

    @property
    def pixel_height(self) -> int:
        return self.height * self.scale

    def _index(self, cell_x: int, cell_y: int) -> int:
        return cell_y * self.width + cell_x

    def in_bounds(self, cell_x: int, cell_y: int) -> bool:
        return 0 <= cell_x < self.width and 0 <= cell_y < self.height

    def world_to_cell(self, point: GridPoint) -> tuple[int, int]:
        local_x, local_y = self.world_to_grid(point)
        return math.floor(local_x), math.floor(local_y)

    def world_to_grid(self, point: GridPoint) -> tuple[float, float]:
        delta_x = point.x - self.origin_x
        delta_y = point.y - self.origin_y
        local_x = self._origin_cos * delta_x + self._origin_sin * delta_y
        local_y = -self._origin_sin * delta_x + self._origin_cos * delta_y
        return local_x / self.resolution, local_y / self.resolution

    def cell_to_world(self, cell_x: int, cell_y: int) -> GridPoint:
        if not self.in_bounds(cell_x, cell_y):
            raise RouteValidationError(f"cell ({cell_x}, {cell_y}) is outside the map")
        local_x = (cell_x + 0.5) * self.resolution
        local_y = (cell_y + 0.5) * self.resolution
        return GridPoint(
            self.origin_x + self._origin_cos * local_x - self._origin_sin * local_y,
            self.origin_y + self._origin_sin * local_x + self._origin_cos * local_y,
        )

    def pixel_to_world(self, x_px: int, y_px: int) -> GridPoint:
        if not (0 <= x_px < self.pixel_width and 0 <= y_px < self.pixel_height):
            raise RouteValidationError(f"pixel ({x_px}, {y_px}) is outside the map image")
        cell_x = x_px // self.scale
        cell_y = self.height - 1 - (y_px // self.scale)
        return self.cell_to_world(cell_x, cell_y)

    def world_to_pixel(self, point: GridPoint) -> tuple[int, int]:
        cell_x, cell_y = self.world_to_cell(point)
        if not self.in_bounds(cell_x, cell_y):
            raise RouteValidationError(
                f"point ({point.x:.3f}, {point.y:.3f}) is outside the map"
            )
        return (
            cell_x * self.scale + self.scale // 2,
            (self.height - 1 - cell_y) * self.scale + self.scale // 2,
        )

    def is_free(self, point: GridPoint) -> bool:
        cell = self.world_to_cell(point)
        return self.in_bounds(*cell) and not self._blocked[self._index(*cell)]

    def validate_point(self, point: GridPoint, label: str = "waypoint") -> None:
        cell = self.world_to_cell(point)
        if not self.in_bounds(*cell):
            raise RouteValidationError(
                f"{label} ({point.x:.3f}, {point.y:.3f}) is outside the map"
            )
        if self._blocked[self._index(*cell)]:
            raise RouteValidationError(
                f"{label} ({point.x:.3f}, {point.y:.3f}) is not in free space"
            )

    def connected_component(
        self, start: GridPoint, seed_max_distance: float
    ) -> tuple[frozenset[tuple[int, int]], GridPoint]:
        """Return the four-connected free component nearest the robot position."""
        start_cell = self.world_to_cell(start)
        if not self.in_bounds(*start_cell):
            raise RouteValidationError(
                f"robot start ({start.x:.3f}, {start.y:.3f}) is outside the map"
            )
        if not self._blocked[self._index(*start_cell)]:
            seed_cell = start_cell
            seed = start
        else:
            nearest = self._nearest_free_cell(start, seed_max_distance)
            if nearest is None:
                raise RouteValidationError(
                    "robot start has no free connectivity seed within "
                    f"{seed_max_distance:.2f} m"
                )
            seed_cell, seed = nearest

        pending = deque([seed_cell])
        visited = {seed_cell}
        while pending:
            cell_x, cell_y = pending.popleft()
            for neighbor in (
                (cell_x - 1, cell_y),
                (cell_x + 1, cell_y),
                (cell_x, cell_y - 1),
                (cell_x, cell_y + 1),
            ):
                if (
                    neighbor in visited
                    or not self.in_bounds(*neighbor)
                    or self._blocked[self._index(*neighbor)]
                ):
                    continue
                visited.add(neighbor)
                pending.append(neighbor)
        return frozenset(visited), seed

    def snap_to_component(
        self,
        point: GridPoint,
        component: frozenset[tuple[int, int]],
        maximum_distance: float,
        label: str = "waypoint",
    ) -> GridPoint:
        """Keep a connected free point or snap it to the nearest component cell."""
        cell = self.world_to_cell(point)
        if not self.in_bounds(*cell):
            raise RouteValidationError(
                f"{label} ({point.x:.3f}, {point.y:.3f}) is outside the map"
            )
        if cell in component:
            return point
        if maximum_distance < 0.0:
            raise ValueError("maximum snap distance must be non-negative")

        radius_cells = math.ceil(maximum_distance / self.resolution)
        candidates: list[tuple[float, int, int, GridPoint]] = []
        for cell_y in range(
            max(0, cell[1] - radius_cells),
            min(self.height, cell[1] + radius_cells + 1),
        ):
            for cell_x in range(
                max(0, cell[0] - radius_cells),
                min(self.width, cell[0] + radius_cells + 1),
            ):
                if (cell_x, cell_y) not in component:
                    continue
                candidate = self.cell_to_world(cell_x, cell_y)
                distance = math.dist(
                    (point.x, point.y), (candidate.x, candidate.y)
                )
                if distance <= maximum_distance + 1e-9:
                    candidates.append((distance, cell_y, cell_x, candidate))
        if not candidates:
            raise RouteValidationError(
                f"{label} has no connected free point within {maximum_distance:.2f} m"
            )
        return min(candidates, key=lambda value: value[:3])[3]

    def render(self, start: GridPoint, goal: GridPoint) -> bytes:
        image = Image.new("RGB", (self.width, self.height), "white")
        pixels = image.load()
        for cell_y in range(self.height):
            image_y = self.height - 1 - cell_y
            for cell_x in range(self.width):
                index = self._index(cell_x, cell_y)
                if self._inflated_avoid[index]:
                    pixels[cell_x, image_y] = (230, 126, 34)
                elif self._pushable[index] and self._blocked[index]:
                    pixels[cell_x, image_y] = (32, 122, 220)
                elif self._blocked[index]:
                    pixels[cell_x, image_y] = (0, 0, 0)
        nearest = getattr(getattr(Image, "Resampling", Image), "NEAREST")
        image = image.resize(
            (self.pixel_width, self.pixel_height), resample=nearest
        )
        draw = ImageDraw.Draw(image)
        radius = max(3, self.scale * 2)
        for point, color in ((start, (0, 180, 0)), (goal, (220, 0, 0))):
            x_px, y_px = self.world_to_pixel(point)
            draw.ellipse(
                (x_px - radius, y_px - radius, x_px + radius, y_px + radius),
                fill=color,
                outline=(255, 255, 255),
                width=max(1, self.scale // 2),
            )
        output = io.BytesIO()
        image.save(output, format="PNG")
        return output.getvalue()

    def _nearest_free_cell(
        self, point: GridPoint, maximum_distance: float
    ) -> tuple[tuple[int, int], GridPoint] | None:
        cell = self.world_to_cell(point)
        radius_cells = math.ceil(maximum_distance / self.resolution)
        candidates: list[tuple[float, int, int, GridPoint]] = []
        for cell_y in range(
            max(0, cell[1] - radius_cells),
            min(self.height, cell[1] + radius_cells + 1),
        ):
            for cell_x in range(
                max(0, cell[0] - radius_cells),
                min(self.width, cell[0] + radius_cells + 1),
            ):
                if self._blocked[self._index(cell_x, cell_y)]:
                    continue
                candidate = self.cell_to_world(cell_x, cell_y)
                distance = math.dist(
                    (point.x, point.y), (candidate.x, candidate.y)
                )
                if distance <= maximum_distance + 1e-9:
                    candidates.append((distance, cell_y, cell_x, candidate))
        if not candidates:
            return None
        _, cell_y, cell_x, candidate = min(
            candidates, key=lambda value: value[:3]
        )
        return (cell_x, cell_y), candidate

    def _paint_disk(self, target: list[bool], x: float, y: float, radius: float) -> None:
        center_x, center_y = self.world_to_grid(GridPoint(x, y))
        radius_cells = radius / self.resolution
        for cell_y in range(
            math.floor(center_y - radius_cells),
            math.floor(center_y + radius_cells) + 1,
        ):
            for cell_x in range(
                math.floor(center_x - radius_cells),
                math.floor(center_x + radius_cells) + 1,
            ):
                if not self.in_bounds(cell_x, cell_y):
                    continue
                nearest_x = min(max(center_x, cell_x), cell_x + 1.0)
                nearest_y = min(max(center_y, cell_y), cell_y + 1.0)
                if math.hypot(
                    center_x - nearest_x, center_y - nearest_y
                ) <= radius_cells + 1e-12:
                    target[self._index(cell_x, cell_y)] = True

    def _paint_box(
        self,
        target: list[bool],
        center_x: float,
        center_y: float,
        size_x: float,
        size_y: float,
    ) -> None:
        corners = [
            self.world_to_grid(GridPoint(x, y))
            for x, y in (
                (center_x - size_x / 2.0, center_y - size_y / 2.0),
                (center_x - size_x / 2.0, center_y + size_y / 2.0),
                (center_x + size_x / 2.0, center_y - size_y / 2.0),
                (center_x + size_x / 2.0, center_y + size_y / 2.0),
            )
        ]
        minimum_x = math.floor(min(point[0] for point in corners))
        maximum_x = math.floor(max(point[0] for point in corners))
        minimum_y = math.floor(min(point[1] for point in corners))
        maximum_y = math.floor(max(point[1] for point in corners))
        for cell_y in range(minimum_y, maximum_y + 1):
            for cell_x in range(minimum_x, maximum_x + 1):
                if self.in_bounds(cell_x, cell_y):
                    target[self._index(cell_x, cell_y)] = True

    def _inflate(self, source: Sequence[bool], radius: float) -> list[bool]:
        cells = max(0, math.ceil(max(0.0, radius) / self.resolution))
        if cells == 0:
            return list(source)
        offsets = [
            (dx, dy)
            for dy in range(-cells, cells + 1)
            for dx in range(-cells, cells + 1)
            if math.hypot(dx, dy) * self.resolution <= radius + 1e-9
        ]
        output = list(source)
        for cell_y in range(self.height):
            for cell_x in range(self.width):
                if not source[self._index(cell_x, cell_y)]:
                    continue
                for dx, dy in offsets:
                    target = cell_x + dx, cell_y + dy
                    if self.in_bounds(*target):
                        output[self._index(*target)] = True
        return output
