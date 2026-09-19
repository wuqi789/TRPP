"""VLM waypoint proposal with deterministic point sanitization."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Sequence

from .artifacts import ArtifactStore
from .grid import GridMap, GridPoint, RouteValidationError
from .vlm import (
    VLMConfigurationError,
    VLMError,
    VLMTransportError,
    parse_waypoints,
)


class RoutePlanningError(RuntimeError):
    pass


class VLMProviderUnavailable(RoutePlanningError):
    pass


class VLMResponseError(RoutePlanningError):
    pass


@dataclass(frozen=True)
class PlannedRoute:
    waypoints: tuple[GridPoint, ...]
    adjusted_anchors: tuple[GridPoint, ...] = ()
    response_attempts: int = 0


class RoutePlanner:
    def __init__(
        self,
        vlm,
        artifacts: ArtifactStore,
        *,
        response_retries: int = 1,
        transport_attempts: int = 3,
        maximum_segment_points: int = 24,
        maximum_route_points: int = 64,
        deduplicate_distance: float = 0.05,
        snap_max_distance: float = 0.50,
        transport_retry_backoff: float = 1.0,
    ) -> None:
        self.vlm = vlm
        self.artifacts = artifacts
        self.response_retries = max(0, int(response_retries))
        self.transport_attempts = max(1, int(transport_attempts))
        self.maximum_segment_points = int(maximum_segment_points)
        self.maximum_route_points = int(maximum_route_points)
        self.deduplicate_distance = float(deduplicate_distance)
        self.snap_max_distance = float(snap_max_distance)
        self.transport_retry_backoff = max(0.0, float(transport_retry_backoff))

    def plan(
        self,
        task_id: str,
        grid: GridMap,
        start: GridPoint,
        anchors: Sequence[GridPoint],
        *,
        attempt_callback: Callable[[int], None] | None = None,
    ) -> PlannedRoute:
        if not anchors:
            raise RoutePlanningError("route requires at least the final goal")

        validation_log: list[dict[str, object]] = []
        try:
            component, component_seed = grid.connected_component(
                start, self.snap_max_distance
            )
        except RouteValidationError:
            validation_log.append({
                "status": "connectivity_failed",
                "reason": "ROUTE_CONNECTIVITY_FAILED",
            })
            self.artifacts.write_json(task_id, "validation.json", validation_log)
            raise RoutePlanningError("ROUTE_CONNECTIVITY_FAILED") from None
        if component_seed != start:
            validation_log.append({
                "status": "connectivity_seed_adjusted",
                "original": [start.x, start.y],
                "adjusted": [component_seed.x, component_seed.y],
            })

        adjusted_anchors: list[GridPoint] = []
        for index, anchor in enumerate(anchors):
            try:
                adjusted = grid.snap_to_component(
                    anchor,
                    component,
                    self.snap_max_distance,
                    f"anchor[{index}]",
                )
            except RouteValidationError:
                validation_log.append({
                    "kind": "anchor",
                    "index": index,
                    "status": "rejected",
                    "original": [anchor.x, anchor.y],
                    "reason": "ROUTE_ANCHOR_INVALID",
                })
                self.artifacts.write_json(
                    task_id, "validation.json", validation_log
                )
                raise RoutePlanningError("ROUTE_ANCHOR_INVALID") from None
            adjusted_anchors.append(adjusted)
            validation_log.append(self._point_record(
                "anchor", index, anchor, adjusted
            ))

        route: list[GridPoint] = []
        segment_start = start
        response_attempt_count = 0
        for segment_index, endpoint in enumerate(adjusted_anchors):
            image = grid.render(segment_start, endpoint)
            self.artifacts.write_bytes(
                task_id, f"segment_{segment_index:02d}_map.png", image
            )
            feedback = ""
            accepted: list[GridPoint] | None = None
            last_error = "VLM did not return a usable response"

            for response_attempt in range(self.response_retries + 1):
                response_attempt_count += 1
                if attempt_callback is not None:
                    attempt_callback(response_attempt_count)
                try:
                    raw = self._infer_with_transport_retries(
                        task_id,
                        segment_index,
                        response_attempt,
                        image,
                        self._request(grid, segment_start, endpoint),
                        feedback,
                        validation_log,
                    )
                    # Provider output can contain private prompts or remote
                    # metadata; persist only bounded receipt metadata.
                    self.artifacts.write_json(
                        task_id,
                        f"segment_{segment_index:02d}_response_{response_attempt:02d}.json",
                        {"status": "received", "bytes": len(raw.encode("utf-8"))},
                    )
                    pixels = parse_waypoints(raw, self.maximum_segment_points)
                    candidates, point_records = self._sanitize_pixels(
                        grid, component, segment_start, pixels
                    )
                    if candidates and math.dist(
                        (candidates[-1].x, candidates[-1].y),
                        (endpoint.x, endpoint.y),
                    ) < self.deduplicate_distance:
                        removed = candidates.pop()
                        point_records.append({
                            "status": "deduplicated_near_anchor",
                            "point": [removed.x, removed.y],
                        })
                    accepted = candidates + [endpoint]
                    validation_log.append({
                        "segment": segment_index,
                        "response_attempt": response_attempt,
                        "status": "accepted",
                        "transport": getattr(self.vlm, "last_transport", "unknown"),
                        "points": point_records,
                        "fifo_points": [[point.x, point.y] for point in accepted],
                        "segment_collision_validation": "disabled",
                    })
                    break
                except VLMConfigurationError:
                    validation_log.append({
                        "segment": segment_index,
                        "response_attempt": response_attempt,
                        "status": "configuration_failed",
                        "reason": "VLM_PROVIDER_CONFIGURATION_FAILED",
                    })
                    self.artifacts.write_json(
                        task_id, "validation.json", validation_log
                    )
                    raise
                except (VLMError, RouteValidationError):
                    last_error = "VLM_RESPONSE_INVALID"
                    feedback = "response_contract_rejected"
                    validation_log.append({
                        "segment": segment_index,
                        "response_attempt": response_attempt,
                        "status": "response_rejected",
                        "reason": "VLM_RESPONSE_INVALID",
                    })

            if accepted is None:
                self.artifacts.write_json(
                    task_id, "validation.json", validation_log
                )
                raise VLMResponseError(
                    f"segment {segment_index} exhausted VLM response attempts: {last_error}"
                )
            route.extend(accepted)
            if len(route) > self.maximum_route_points:
                self.artifacts.write_json(
                    task_id, "validation.json", validation_log
                )
                raise RoutePlanningError(
                    f"route exceeds the task limit of {self.maximum_route_points} points"
                )
            segment_start = endpoint

        self.artifacts.write_json(task_id, "validation.json", validation_log)
        self.artifacts.write_json(
            task_id,
            "route.json",
            {
                "frame_id": grid.frame_id,
                "start": [start.x, start.y],
                "waypoints": [[point.x, point.y] for point in route],
                "segment_collision_validation": "disabled",
            },
        )
        return PlannedRoute(
            tuple(route), tuple(adjusted_anchors), response_attempt_count
        )

    def _infer_with_transport_retries(
        self,
        task_id: str,
        segment_index: int,
        response_attempt: int,
        image: bytes,
        request: dict[str, object],
        feedback: str,
        validation_log: list[dict[str, object]],
    ) -> str:
        last_error = ""
        for transport_attempt in range(self.transport_attempts):
            try:
                raw = self.vlm.infer(
                    image,
                    request,
                    feedback={"reason": feedback} if feedback else None,
                )
                validation_log.append({
                    "segment": segment_index,
                    "response_attempt": response_attempt,
                    "transport_attempt": transport_attempt + 1,
                    "status": "transport_succeeded",
                    "transport": getattr(self.vlm, "last_transport", "unknown"),
                })
                return raw
            except VLMTransportError:
                last_error = "VLM_PROVIDER_UNAVAILABLE"
                validation_log.append({
                    "segment": segment_index,
                    "response_attempt": response_attempt,
                    "transport_attempt": transport_attempt + 1,
                    "status": "transport_failed",
                    "reason": "VLM_PROVIDER_UNAVAILABLE",
                })
                if transport_attempt + 1 < self.transport_attempts:
                    time.sleep(
                        self.transport_retry_backoff * (transport_attempt + 1)
                    )
        self.artifacts.write_json(task_id, "validation.json", validation_log)
        raise VLMProviderUnavailable("VLM_PROVIDER_UNAVAILABLE")

    def _sanitize_pixels(
        self,
        grid: GridMap,
        component: frozenset[tuple[int, int]],
        start: GridPoint,
        pixels: Sequence[tuple[int, int]],
    ) -> tuple[list[GridPoint], list[dict[str, object]]]:
        output: list[GridPoint] = []
        records: list[dict[str, object]] = []
        previous = start
        for index, pixel in enumerate(pixels):
            original = grid.pixel_to_world(*pixel)
            adjusted = grid.snap_to_component(
                original,
                component,
                self.snap_max_distance,
                f"waypoint[{index}]",
            )
            record = self._point_record("waypoint", index, original, adjusted)
            record["pixel"] = list(pixel)
            if math.dist(
                (adjusted.x, adjusted.y), (previous.x, previous.y)
            ) < self.deduplicate_distance:
                record["status"] = "deduplicated"
                records.append(record)
                continue
            output.append(adjusted)
            previous = adjusted
            records.append(record)
        return output, records

    @staticmethod
    def _point_record(
        kind: str, index: int, original: GridPoint, adjusted: GridPoint
    ) -> dict[str, object]:
        return {
            "kind": kind,
            "index": index,
            "status": "snapped" if adjusted != original else "kept",
            "original": [original.x, original.y],
            "adjusted": [adjusted.x, adjusted.y],
        }

    @staticmethod
    def _request(grid: GridMap, start: GridPoint, goal: GridPoint) -> dict[str, object]:
        start_px = grid.world_to_pixel(start)
        goal_px = grid.world_to_pixel(goal)
        return {
            "contract": "scout.route-waypoints.v1",
            "image": {
                "width_px": grid.pixel_width,
                "height_px": grid.pixel_height,
                "origin": "top_left",
            },
            "start_px": [start_px[0], start_px[1]],
            "goal_px": [goal_px[0], goal_px[1]],
            "response_schema": {"waypoints": [{"x_px": "integer", "y_px": "integer"}]},
        }
