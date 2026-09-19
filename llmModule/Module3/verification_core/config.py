"""Verification configuration and mock occupancy-grid model."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml


@dataclass(frozen=True)
class OccupancyGrid:
    width: int
    height: int
    resolution: float = 1.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    occupied: frozenset[tuple[int, int]] = frozenset()

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "OccupancyGrid":
        occupied = frozenset((int(cell[0]), int(cell[1])) for cell in value.get("occupied", []))
        grid = cls(
            width=int(value.get("width", 0)),
            height=int(value.get("height", 0)),
            resolution=float(value.get("resolution", 1.0)),
            origin_x=float(value.get("origin_x", 0.0)),
            origin_y=float(value.get("origin_y", 0.0)),
            occupied=occupied,
        )
        if grid.width <= 0 or grid.height <= 0 or grid.resolution <= 0:
            raise ValueError("occupancy grid width, height, and resolution must be positive")
        return grid

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (
            math.floor((x - self.origin_x) / self.resolution),
            math.floor((y - self.origin_y) / self.resolution),
        )

def load_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError("verification configuration root must be a mapping")
    return data
