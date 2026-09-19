#!/usr/bin/env python3
"""Local route-adapter smoke test."""

from route_planning import ArtifactStore, GridMap, GridPoint, RoutePlanner
from semantic_navigation_adapters import MockVLMAdapter


def main() -> int:
    grid = GridMap(10, 10, 1.0, 0.0, 0.0, [0] * 100, inflation_radius=0.10, scale=4)
    result = RoutePlanner(MockVLMAdapter(), ArtifactStore("runtime/route_smoke")).plan(
        "smoke", grid, GridPoint(1.5, 1.5), [GridPoint(8.5, 1.5)]
    )
    print(f"route_points={len(result.waypoints)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
