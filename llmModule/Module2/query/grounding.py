"""Resolve a semantic navigation intent to exactly one scene entity."""

from semantic_graph import SemanticGraph, SemanticNode

from .entity_query import query_entities
from .relation_query import query_relation


class GroundingError(ValueError):
    """Raised when an intent cannot be grounded without guessing."""


def ground_entity(
    graph: SemanticGraph,
    goal_object: str,
    reference_object: str = "",
    relation: str = "",
) -> SemanticNode:
    candidates = query_entities(graph, goal_object)
    if not candidates:
        raise GroundingError(f"No semantic entity found for '{goal_object}'")
    if len(candidates) == 1:
        return candidates[0]
    if not reference_object.strip() or not relation.strip():
        raise GroundingError(
            f"Semantic entity '{goal_object}' is ambiguous ({len(candidates)} matches); "
            "reference_object and relation are required"
        )

    matches = query_relation(graph, goal_object, relation, reference_object)
    candidate_ids = {candidate.id for candidate in candidates}
    matches = [match for match in matches if match.id in candidate_ids]
    if len(matches) != 1:
        raise GroundingError(
            f"Relation '{goal_object} {relation} {reference_object}' resolved to "
            f"{len(matches)} entities; expected exactly one"
        )
    return matches[0]
