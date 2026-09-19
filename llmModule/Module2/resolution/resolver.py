"""ROS-independent intent grounding and executable-constraint resolution."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any, Mapping

from matching import EntityMatch, EntityMatcher
from providers import SemanticMapProvider
from semantic_graph import Relation, SemanticNode


class ResolutionError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class ConstraintResolution:
    type: str
    source_text: str
    entity_id: str
    canonical_name: str
    pose: tuple[float, float, float]
    radius: float
    order: int


@dataclass(frozen=True)
class GoalResolution:
    request_id: str
    map_revision: str
    goal_id: str
    canonical_name: str
    match_method: str
    similarity: float
    pose: tuple[float, float, float]
    topology_context: tuple[str, ...]
    constraints: tuple[ConstraintResolution, ...]
    strategy: str


class SemanticResolver:
    def __init__(
        self,
        provider: SemanticMapProvider,
        matcher: EntityMatcher,
        *,
        start_node: str = "jackal_robot",
        robot_radius: float = 0.333,
        safety_margin: float = 0.10,
        default_entity_radius: float = 0.25,
    ) -> None:
        self.provider = provider
        self.matcher = matcher
        self.start_node = start_node
        self.robot_radius = float(robot_radius)
        self.safety_margin = float(safety_margin)
        self.default_entity_radius = float(default_entity_radius)

    def resolve(self, intent: Mapping[str, Any]) -> GoalResolution:
        goal_text = str(intent.get("goal_object", ""))
        goal_node = self.provider.entity(goal_text.strip())
        goal_match = (
            EntityMatch(goal_node.name, "id", 1.0)
            if goal_node is not None
            else self._match(goal_text)
        )
        reference_match = None
        reference_node = None
        reference = str(intent.get("reference_object", "")).strip()
        if reference:
            reference_node = self.provider.entity(reference)
            reference_match = (
                EntityMatch(reference_node.name, "id", 1.0)
                if reference_node is not None
                else self._match(reference)
            )
        goal = self._select_instance(
            goal_match.canonical_name,
            reference_match.canonical_name if reference_match else "",
            str(intent.get("relation", "")),
            selected=goal_node,
            reference_node=reference_node,
        )
        constraints = tuple(
            self._resolve_constraint(value, order)
            for order, value in enumerate(intent.get("constraints", []))
        )
        pose = self.provider.navigation_pose(goal)
        topology = tuple(self.provider.topology_path(self.start_node, goal.id))
        return GoalResolution(
            request_id=str(intent.get("request_id", "")),
            map_revision=self.provider.revision,
            goal_id=goal.id,
            canonical_name=goal_match.canonical_name,
            match_method=goal_match.method,
            similarity=goal_match.similarity,
            pose=pose,
            topology_context=topology,
            constraints=constraints,
            strategy=str(intent.get("strategy", "shortest")) or "shortest",
        )

    def _match(self, query: str) -> EntityMatch:
        try:
            return self.matcher.match(query)
        except ValueError:
            raise ResolutionError(
                "SEMANTIC_MATCH_FAILED",
                f"Could not match semantic entity '{query}'",
            ) from None
        except RuntimeError:
            raise ResolutionError(
                "SEMANTIC_MATCHER_UNAVAILABLE",
                f"Could not match semantic entity '{query}'",
            ) from None

    def _select_instance(
        self,
        canonical: str,
        reference: str = "",
        relation: str = "",
        *,
        selected: SemanticNode | None = None,
        reference_node: SemanticNode | None = None,
    ) -> SemanticNode:
        candidates = [selected] if selected is not None else self.provider.instances(canonical)
        if not candidates:
            raise ResolutionError("ENTITY_NOT_FOUND", f"No instances of '{canonical}' exist")
        if not reference and not relation.strip() and len(candidates) == 1:
            return candidates[0]
        if not reference or not relation.strip():
            raise ResolutionError(
                "ENTITY_AMBIGUOUS",
                f"'{canonical}' has {len(candidates)} instances and requires a relation",
            )
        try:
            relation_value = Relation.parse(relation).value
        except ValueError:
            raise ResolutionError(
                "UNSUPPORTED_RELATION", "Relation is not supported"
            ) from None
        references = (
            [reference_node]
            if reference_node is not None
            else self.provider.instances(reference)
        )
        if not references:
            raise ResolutionError(
                "REFERENCE_NOT_FOUND", f"No instances of '{reference}' exist"
            )
        matches = [
            candidate
            for candidate in candidates
            if any(
                self.provider.graph.has_relation(candidate.id, relation_value, target.id)
                for target in references
            )
        ]
        if len(matches) != 1:
            raise ResolutionError(
                "RELATION_AMBIGUOUS",
                f"'{canonical} {relation_value} {reference}' resolved to {len(matches)} instances",
            )
        return matches[0]

    def _resolve_constraint(self, raw: Any, order: int) -> ConstraintResolution:
        text = str(raw).strip()
        match = re.fullmatch(r"(avoid|via)\s+(.+)", text, re.IGNORECASE)
        if not match:
            raise ResolutionError(
                "UNSUPPORTED_CONSTRAINT",
                f"Unsupported constraint '{text}'; expected 'avoid <entity>' or 'via <entity>'",
            )
        constraint_type = match.group(1).lower()
        entity_text = match.group(2).strip()
        node = self.provider.entity(entity_text) or self._property_landmark(entity_text)
        entity_match = (
            EntityMatch(
                node.name,
                "id" if node.id.casefold() == entity_text.casefold() else "semantic_property",
                1.0,
            )
            if node is not None
            else self._match(entity_text)
        )
        node = self._select_instance(entity_match.canonical_name, selected=node)
        pose = self.provider.navigation_pose(node)
        footprint = float(node.properties.get("footprint_radius", self.default_entity_radius))
        radius = (
            footprint + self.robot_radius + self.safety_margin
            if constraint_type == "avoid"
            else 0.0
        )
        return ConstraintResolution(
            type=constraint_type,
            source_text=text,
            entity_id=node.id,
            canonical_name=entity_match.canonical_name,
            pose=pose,
            radius=radius,
            order=order,
        )

    def _property_landmark(self, query: str) -> SemanticNode | None:
        normalized = " ".join(re.findall(r"[\w]+", query.casefold().replace("_", " ")))
        curtain_terms = {"curtain", "fabric curtain", "door curtain", "门帘"}
        if normalized not in curtain_terms:
            return None
        matches = [
            node
            for node in self.provider.graph.nodes()
            if "curtain" in str(node.properties.get("collision_proxy_state", "")).casefold()
        ]
        if len(matches) != 1:
            raise ResolutionError(
                "LANDMARK_AMBIGUOUS",
                f"Semantic landmark '{query}' resolved to {len(matches)} map entities",
            )
        return matches[0]
