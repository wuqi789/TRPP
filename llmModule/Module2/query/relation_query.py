"""Queries over explicit semantic relations."""

from semantic_graph import Relation, SemanticGraph, SemanticNode

from .entity_query import resolve_ids


def query_relation(
    graph: SemanticGraph, source: str, relation: str, target: str
) -> list[SemanticNode]:
    relation_value = Relation.parse(relation).value
    source_ids = resolve_ids(graph, source)
    target_ids = resolve_ids(graph, target)
    matched: list[SemanticNode] = []
    for source_id in source_ids:
        if any(graph.has_relation(source_id, relation_value, target_id) for target_id in target_ids):
            node = graph.get_node(source_id)
            if node is not None:
                matched.append(node)
    return matched

