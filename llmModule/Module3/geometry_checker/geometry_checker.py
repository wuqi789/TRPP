"""Mock occupancy-grid goal validation."""

from __future__ import annotations

from typing import Any, Mapping

from verification_core.config import OccupancyGrid


class GeometryChecker:
    """Grid checker; a Nav2 ComputePathToPose adapter can replace it later."""

    def check_goal_pose(
        self, position: Mapping[str, Any], occupancy_grid: OccupancyGrid
    ) -> dict[str, Any]:
        try:
            x, y = float(position["x"]), float(position["y"])
        except (KeyError, TypeError, ValueError):
            return {"valid": False, "reason": "invalid_position"}
        cell_x, cell_y = occupancy_grid.world_to_cell(x, y)
        in_bounds = 0 <= cell_x < occupancy_grid.width and 0 <= cell_y < occupancy_grid.height
        if not in_bounds:
            return {"valid": False, "reason": "out_of_bounds"}
        if (cell_x, cell_y) in occupancy_grid.occupied:
            return {"valid": False, "reason": "occupied"}
        return {"valid": True, "reason": ""}
