"""Mock dynamic-safety checker."""

from __future__ import annotations

from typing import Any, Mapping


class DynamicChecker:
    """Stable interface reserved for costmap, LaserScan, and camera adapters."""

    def check(self, environment_state: Mapping[str, Any] | None) -> dict[str, bool]:
        state = environment_state or {}
        return {"safe": bool(state.get("safe", True))}

