"""Distance-and-dwell FIFO progress state machine."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable

from route_planning.grid import GridPoint


@dataclass(frozen=True)
class FifoEvent:
    kind: str
    target: GridPoint | None = None
    code: str = ""
    message: str = ""


class FifoTracker:
    def __init__(
        self,
        *,
        arrival_distance: float = 0.15,
        arrival_hold_seconds: float = 0.5,
        segment_timeout: float = 120.0,
        no_progress_timeout: float = 30.0,
        progress_distance: float = 0.05,
        waypoint_pass_lateral_distance: float = 0.75,
        watchdog_enabled: bool = True,
    ) -> None:
        self.arrival_distance = float(arrival_distance)
        self.arrival_hold_seconds = float(arrival_hold_seconds)
        self.segment_timeout = float(segment_timeout)
        self.no_progress_timeout = float(no_progress_timeout)
        self.progress_distance = float(progress_distance)
        self.waypoint_pass_lateral_distance = float(
            waypoint_pass_lateral_distance
        )
        self.watchdog_enabled = bool(watchdog_enabled)
        self.pending: deque[GridPoint] = deque()
        self.current: GridPoint | None = None
        self.segment_origin: GridPoint | None = None
        self.segment_started = 0.0
        self.inside_since: float | None = None
        self.progress_position: GridPoint | None = None
        self.progress_at = 0.0
        self.paused_at: float | None = None

    def start(
        self, points: Iterable[GridPoint], robot: GridPoint, now: float
    ) -> FifoEvent:
        self.pending = deque(points)
        if not self.pending:
            return FifoEvent("COMPLETE")
        self._activate(self.pending.popleft(), robot, now)
        return FifoEvent("TARGET", self.current)

    def tick(self, robot: GridPoint, now: float) -> FifoEvent:
        if self.paused_at is not None:
            return FifoEvent("NONE", self.current)
        target = self.current
        if target is None:
            return FifoEvent("COMPLETE")
        distance = math.dist((robot.x, robot.y), (target.x, target.y))
        if self._passed_intermediate_waypoint(robot):
            self._activate(self.pending.popleft(), robot, now)
            return FifoEvent("TARGET", self.current)
        if self.watchdog_enabled and now - self.segment_started >= self.segment_timeout:
            return FifoEvent(
                "FAILED", code="SEGMENT_TIMEOUT",
                message=(
                    f"Current segment timed out after {now - self.segment_started:.1f}s; "
                    f"robot=({robot.x:.3f}, {robot.y:.3f}), "
                    f"target=({target.x:.3f}, {target.y:.3f}), distance={distance:.3f}m"
                ),
            )
        if self.progress_position is None or math.dist(
            (robot.x, robot.y),
            (self.progress_position.x, self.progress_position.y),
        ) >= self.progress_distance:
            self.progress_position = robot
            self.progress_at = now
        elif self.watchdog_enabled and now - self.progress_at >= self.no_progress_timeout:
            return FifoEvent(
                "FAILED", code="NO_PROGRESS",
                message=(
                    f"Robot moved less than {self.progress_distance:.3f}m for "
                    f"{now - self.progress_at:.1f}s; robot=({robot.x:.3f}, {robot.y:.3f}), "
                    f"target=({target.x:.3f}, {target.y:.3f}), distance={distance:.3f}m"
                ),
            )
        if distance > self.arrival_distance:
            self.inside_since = None
            return FifoEvent("NONE", target)
        if self.inside_since is None:
            self.inside_since = now
            return FifoEvent("NONE", target)
        if now - self.inside_since < self.arrival_hold_seconds:
            return FifoEvent("NONE", target)
        if not self.pending:
            self.current = None
            return FifoEvent("COMPLETE")
        self._activate(self.pending.popleft(), robot, now)
        return FifoEvent("TARGET", self.current)

    def distance_remaining(self, robot: GridPoint) -> float:
        if self.current is None:
            return 0.0
        return math.dist((robot.x, robot.y), (self.current.x, self.current.y))

    def clear(self) -> None:
        self.pending.clear()
        self.current = None
        self.segment_origin = None
        self.inside_since = None
        self.paused_at = None

    def pause(self, now: float) -> None:
        """Freeze segment, progress, and arrival dwell clocks."""
        if self.current is not None and self.paused_at is None:
            self.paused_at = float(now)

    def resume(self, now: float) -> None:
        """Shift every active deadline by the paused duration."""
        if self.paused_at is None:
            return
        duration = max(0.0, float(now) - self.paused_at)
        self.segment_started += duration
        self.progress_at += duration
        if self.inside_since is not None:
            self.inside_since += duration
        self.paused_at = None

    def _activate(self, target: GridPoint, robot: GridPoint, now: float) -> None:
        self.current = target
        self.segment_origin = robot
        self.segment_started = now
        self.inside_since = None
        self.progress_position = robot
        self.progress_at = now
        self.paused_at = None

    def _passed_intermediate_waypoint(self, robot: GridPoint) -> bool:
        """Accept a direction waypoint after a bounded local-avoidance detour.

        Intermediate Module4 points indicate route direction rather than final
        destinations. NeuPAN can legitimately pass one laterally while avoiding
        furniture, so waiting for the arrival circle would deadlock the FIFO.
        The final goal still requires the normal distance-and-dwell check.
        """
        if not self.pending or self.current is None or self.segment_origin is None:
            return False
        dx = self.current.x - self.segment_origin.x
        dy = self.current.y - self.segment_origin.y
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return False
        rx = robot.x - self.segment_origin.x
        ry = robot.y - self.segment_origin.y
        along = (rx * dx + ry * dy) / length
        lateral = abs(rx * dy - ry * dx) / length
        distance = math.dist(
            (robot.x, robot.y), (self.current.x, self.current.y)
        )
        return (
            distance > self.arrival_distance
            and along >= length - self.arrival_distance
            and lateral <= self.waypoint_pass_lateral_distance
        )
