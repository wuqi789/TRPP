"""ROS-independent robot hold stability tracking."""

from __future__ import annotations

from dataclasses import dataclass
import math

from route_planning.grid import GridPoint


@dataclass(frozen=True)
class StabilityEvent:
    kind: str


class StabilityTracker:
    def __init__(
        self,
        *,
        movement_distance: float,
        hold_seconds: float,
        timeout: float,
    ) -> None:
        self.movement_distance = float(movement_distance)
        self.hold_seconds = float(hold_seconds)
        self.timeout = float(timeout)
        self.started_at = 0.0
        self.stable_since = 0.0
        self.reference: GridPoint | None = None

    def start(self, position: GridPoint, now: float) -> None:
        self.started_at = now
        self.stable_since = now
        self.reference = position

    def tick(self, position: GridPoint, now: float) -> StabilityEvent:
        if self.reference is None:
            raise RuntimeError("stability tracker has not been started")
        if math.dist(
            (position.x, position.y),
            (self.reference.x, self.reference.y),
        ) >= self.movement_distance:
            self.reference = position
            self.stable_since = now
        if now - self.stable_since >= self.hold_seconds:
            return StabilityEvent("STABLE")
        if now - self.started_at >= self.timeout:
            return StabilityEvent("TIMEOUT")
        return StabilityEvent("WAITING")

