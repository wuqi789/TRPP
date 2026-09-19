"""Four-stage verification pipeline with injected map, geometry, and planner ports."""

from __future__ import annotations

import math
from dataclasses import dataclass

from semantic_navigation_adapters import path_length


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    code: str = ""
    message: str = ""


@dataclass(frozen=True)
class PipelineResult:
    verified: bool
    error_code: str
    message: str
    checks: tuple[Check, ...]
    planning_length: float = 0.0
    planning_time: float = 0.0


class VerificationPipeline:
    CHECK_NAMES = ("entity", "topology", "geometry", "planner")

    def __init__(
        self,
        repository,
        geometry,
        planner,
        mask_manager,
        *,
        start_node: str = "jackal_robot",
        pose_tolerance: float = 0.02,
        planner_timeout: float = 10.0,
        mask_update_timeout: float = 10.0,
        entity_check_enabled: bool = True,
        topology_check_enabled: bool = True,
        geometry_check_enabled: bool = False,
        planner_check_enabled: bool = True,
    ) -> None:
        self.repository = repository
        self.geometry = geometry
        self.planner = planner
        self.mask_manager = mask_manager
        self.start_node = start_node
        self.pose_tolerance = float(pose_tolerance)
        self.planner_timeout = float(planner_timeout)
        self.mask_update_timeout = float(mask_update_timeout)
        self.entity_check_enabled = bool(entity_check_enabled)
        self.topology_check_enabled = bool(topology_check_enabled)
        self.geometry_check_enabled = bool(geometry_check_enabled)
        self.planner_check_enabled = bool(planner_check_enabled)

    def verify(self, resolution) -> PipelineResult:
        checks: list[Check] = []
        if self.entity_check_enabled and not resolution.resolved:
            return self._failed(
                checks,
                "entity",
                resolution.error_code or "RESOLUTION_FAILED",
                resolution.message or "Semantic resolution failed",
            )

        node = self.repository.graph.get_node(resolution.goal_id)
        if self.entity_check_enabled and resolution.map_revision != self.repository.revision:
            return self._failed(
                checks,
                "entity",
                "MAP_REVISION_MISMATCH",
                "Module2 and Module3 are not using the same semantic map revision",
            )
        if self.entity_check_enabled and (
            node is None or node.name != resolution.canonical_name
        ):
            return self._failed(
                checks,
                "entity",
                "ENTITY_NOT_FOUND",
                "Resolved entity does not exist in the verification repository",
            )
        if self.entity_check_enabled:
            expected = self.repository.navigation_pose(node)
            actual = resolution.goal_pose.pose.position
            orientation = resolution.goal_pose.pose.orientation
            actual_yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
            )
            yaw_error = abs(math.atan2(
                math.sin(actual_yaw - expected[2]),
                math.cos(actual_yaw - expected[2]),
            ))
            if (
                math.dist(expected[:2], (actual.x, actual.y)) > self.pose_tolerance
                or yaw_error > self.pose_tolerance
            ):
                return self._failed(
                    checks,
                    "entity",
                    "ENTITY_POSE_MISMATCH",
                    "Resolved goal pose differs from the semantic-map pose",
                )
            checks.append(Check("entity", "PASS"))
        else:
            checks.append(Check(
                "entity", "SKIPPED", "ENTITY_CHECK_DISABLED",
                "Entity verification is disabled by the research valve",
            ))

        if self.topology_check_enabled:
            topology = self.repository.topology_path(self.start_node, resolution.goal_id)
            if not topology:
                return self._failed(
                    checks,
                    "topology",
                    "TOPOLOGY_UNREACHABLE",
                    "No semantic topology path reaches the target",
                )
            checks.append(Check("topology", "PASS"))
        else:
            checks.append(Check(
                "topology", "SKIPPED", "TOPOLOGY_CHECK_DISABLED",
                "Topology verification is disabled by the research valve",
            ))

        poses = [resolution.goal_pose] + [
            value.pose for value in resolution.constraints if value.type.casefold() == "via"
        ]
        avoids = [value for value in resolution.constraints if value.type.casefold() == "avoid"]
        for pose in poses:
            for avoid in avoids:
                distance = math.dist(
                    (pose.pose.position.x, pose.pose.position.y),
                    (avoid.pose.pose.position.x, avoid.pose.pose.position.y),
                )
                if distance <= avoid.radius:
                    return self._failed(
                        checks,
                        "geometry",
                        "CONSTRAINT_CONFLICT",
                        "Goal or via point lies inside an avoid region",
                    )
        if self.geometry_check_enabled:
            for pose in poses:
                geometry = self.geometry.validate_pose(
                    pose.pose.position.x, pose.pose.position.y
                )
                if not geometry.valid:
                    return self._failed(
                        checks, "geometry", geometry.code, geometry.message
                    )
            checks.append(Check("geometry", "PASS"))
        else:
            checks.append(
                Check(
                    "geometry",
                    "SKIPPED",
                    "GEOMETRY_OCCUPANCY_DISABLED",
                    "Costmap pose occupancy validation is disabled at startup",
                )
            )

        if not self.planner_check_enabled:
            checks.append(Check(
                "planner", "SKIPPED", "PLANNER_CHECK_DISABLED",
                "Nav2 path verification is disabled by the research valve",
            ))
            return PipelineResult(
                verified=True,
                error_code="",
                message="Verification passed with one or more research valves open",
                checks=tuple(checks),
            )

        sequence = getattr(self.geometry, "sequence", None)
        try:
            mask_applied = self.mask_manager.apply(
                resolution.constraints, wait=False
            )
        except Exception:
            return self._failed(
                checks, "planner", "KEEPOUT_MASK_FAILED", "Keepout mask update failed"
            )
        if not mask_applied:
            return self._failed(
                checks,
                "planner",
                "KEEPOUT_MASK_UNAVAILABLE",
                "Validation keepout mask is not ready",
            )
        if sequence is not None and not self.geometry.wait_for_update(
            sequence, self.mask_update_timeout
        ):
            self.mask_manager.clear(wait=False)
            return self._failed(
                checks,
                "planner",
                "KEEPOUT_MASK_TIMEOUT",
                "Validation costmap did not absorb the keepout mask",
            )
        try:
            try:
                planned = self.planner.plan(
                    resolution.goal_pose,
                    resolution.constraints,
                    timeout=self.planner_timeout,
                )
            except Exception:
                return self._failed(
                    checks, "planner", "PLANNER_FAILED", "Planner failed"
                )
        finally:
            clear_sequence = getattr(self.geometry, "sequence", None)
            self.mask_manager.clear(wait=False)
            if clear_sequence is not None:
                self.geometry.wait_for_update(
                    clear_sequence, self.mask_update_timeout
                )
        if not planned.success:
            return self._failed(checks, "planner", planned.code, planned.message)
        checks.append(Check("planner", "PASS"))
        bypassed = any(check.status == "SKIPPED" for check in checks)
        return PipelineResult(
            verified=True,
            error_code="",
            message=(
                "Verification passed with one or more research valves open"
                if bypassed
                else "All verification checks passed"
            ),
            checks=tuple(checks),
            planning_length=path_length(planned.path),
            planning_time=planned.planning_time,
        )

    def _failed(
        self,
        checks: list[Check],
        name: str,
        code: str,
        message: str,
    ) -> PipelineResult:
        checks.append(Check(name, "FAIL", code, message))
        completed = {value.name for value in checks}
        for remaining in self.CHECK_NAMES:
            if remaining not in completed:
                checks.append(Check(remaining, "SKIPPED", "DEPENDENCY_FAILED", ""))
        return PipelineResult(False, code, message, tuple(checks))

    def dependency_failure(
        self, name: str, code: str, message: str
    ) -> PipelineResult:
        checks = []
        for check_name in self.CHECK_NAMES:
            if check_name == name:
                checks.append(Check(check_name, "FAIL", code, message))
            else:
                checks.append(
                    Check(check_name, "SKIPPED", "VALIDATION_NOT_READY", "")
                )
        return PipelineResult(False, code, message, tuple(checks))
