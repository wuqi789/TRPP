"""Scene-specific structural filtering for the Scout occupancy map."""

from __future__ import annotations

import math


# The full-room asset contains visual door assemblies even where the passage is
# open. Rasterizing their complete axis-aligned bounds closes the portal in the
# 2D map. door_0000 is replaced by the cloth curtain at runtime; door_0001 is
# the open kitchen assembly visible from the living room.
TRAVERSABLE_DOOR_LEAVES = frozenset({"door_0000", "door_0001"})
NAVIGATION_MIN_Z = 0.02
NAVIGATION_MAX_Z = 0.35
OPEN_KITCHEN_DOOR_CENTER_Y = -0.9007875518798829
# doorsill_0003 spans y=-2.252787..0.451212 in the source asset. Use that
# walkable threshold, rather than the smaller center leaf, as the portal width.
OPEN_KITCHEN_DOOR_HALF_WIDTH = 1.352
OPEN_KITCHEN_DOOR_TOP_Z = 2.12


def is_blocking_other_prim(name: str) -> bool:
    """Return whether an ``other``-group prim needs a blocking AABB proxy."""
    normalized = str(name).strip()
    if normalized.startswith("doorsill_"):
        # Door sills are floor-level thresholds, not wall-height obstacles.
        return False
    return normalized.startswith("door_") and normalized not in TRAVERSABLE_DOOR_LEAVES


def solid_spans_around_openings(
    span_min: float,
    span_max: float,
    openings: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    """Split a wall span into solid sections around traversable openings."""
    clipped = sorted(
        (max(span_min, opening_min), min(span_max, opening_max))
        for opening_min, opening_max in openings
        if opening_max > span_min and opening_min < span_max
    )
    solids = []
    cursor = span_min
    for opening_min, opening_max in clipped:
        if opening_min > cursor:
            solids.append((cursor, opening_min))
        cursor = max(cursor, opening_max)
    if cursor < span_max:
        solids.append((cursor, span_max))
    return tuple(solids)


def intersects_navigation_height(minimum_z: float, maximum_z: float) -> bool:
    """Return whether a 3D proxy belongs in the ground-plane occupancy map."""
    return minimum_z < NAVIGATION_MAX_Z and maximum_z > NAVIGATION_MIN_Z


def world_origin_in_start_frame(
    start_x: float, start_y: float, start_yaw: float
) -> tuple[float, float, float]:
    """Return world-origin coordinates expressed in the robot start frame."""
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    return (
        cosine * -start_x + sine * -start_y,
        -sine * -start_x + cosine * -start_y,
        -start_yaw,
    )
