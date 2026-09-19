"""Ordered coordinator for all navigation verification levels."""

from __future__ import annotations

from typing import Any, Mapping

from dynamic_checker import DynamicChecker
from entity_checker import EntityChecker
from geometry_checker import GeometryChecker
from topology_checker import TopologyChecker

from .config import OccupancyGrid
from .verification_result import VerificationResult


class NavigationVerifier:
    def __init__(
        self,
        config: Mapping[str, Any],
        occupancy_grid: OccupancyGrid,
        start_node: str = "jackal_robot",
    ) -> None:
        self.config = config
        self.occupancy_grid = occupancy_grid
        self.start_node = start_node
        self.entity_checker = EntityChecker()
        self.topology_checker = TopologyChecker()
        self.geometry_checker = GeometryChecker()
        self.dynamic_checker = DynamicChecker()

    def _enabled(self, check_name: str) -> bool:
        settings = self.config.get(check_name, {})
        return bool(settings.get("enabled", True)) if isinstance(settings, Mapping) else True

    def verify_navigation(self, intent: Mapping[str, Any], graph: Any) -> VerificationResult:
        requested_goal = str(intent.get("goal_id") or intent.get("goal_object") or "")
        goal_pose = self._pose(intent)
        goal_id = requested_goal
        if self._enabled("entity_check"):
            entity = self.entity_checker.check(requested_goal, graph)
            if not entity["success"]:
                return self._failure("entity_not_found", goal_id, goal_pose)
            goal_id = entity["candidate_id"]

        if self._enabled("topology_check"):
            topology = self.topology_checker.check(self.start_node, goal_id, graph)
            if not topology["reachable"]:
                return self._failure("topology_unreachable", goal_id, goal_pose)

        if self._enabled("geometry_check"):
            geometry = self.geometry_checker.check_goal_pose(goal_pose, self.occupancy_grid)
            if not geometry["valid"]:
                return self._failure("geometry_invalid", goal_id, goal_pose)

        if self._enabled("dynamic_check"):
            dynamic = self.dynamic_checker.check(intent.get("environment_state"))
            if not dynamic["safe"]:
                return self._failure("dynamic_unsafe", goal_id, goal_pose)

        return VerificationResult(
            verified=True,
            goal_id=goal_id,
            goal_pose=goal_pose,
            explanation="All verification passed",
        )

    @staticmethod
    def _pose(intent: Mapping[str, Any]) -> dict[str, float]:
        raw_pose = intent.get("position", intent.get("goal_pose", {}))
        if not isinstance(raw_pose, Mapping):
            return {}
        try:
            return {
                "x": float(raw_pose["x"]),
                "y": float(raw_pose["y"]),
                "theta": float(raw_pose.get("theta", 0.0)),
            }
        except (KeyError, TypeError, ValueError):
            return {}

    @staticmethod
    def _failure(
        failed_check: str,
        goal_id: str,
        goal_pose: dict[str, float],
    ) -> VerificationResult:
        explanations = {
            "entity_not_found": "Target entity does not exist in the semantic graph",
            "topology_unreachable": "No semantic topology path reaches the target",
            "geometry_invalid": "Goal pose is outside the free occupancy grid",
            "dynamic_unsafe": "Dynamic environment is currently unsafe",
        }
        return VerificationResult(
            verified=False,
            goal_id=goal_id,
            goal_pose=goal_pose,
            failed_checks=(failed_check,),
            explanation=explanations[failed_check],
        )


def verify_navigation(
    intent: Mapping[str, Any], graph: Any, config: Mapping[str, Any], occupancy_grid: OccupancyGrid
) -> VerificationResult:
    """Functional form of the required verify_navigation(intent, graph) workflow."""
    return NavigationVerifier(config, occupancy_grid).verify_navigation(intent, graph)
