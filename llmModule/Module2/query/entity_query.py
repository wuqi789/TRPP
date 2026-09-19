"""Entity lookup by semantic name or exact identifier."""

from semantic_graph import SemanticGraph, SemanticNode


def query_entities(graph: SemanticGraph, entity_name: str) -> list[SemanticNode]:
    term = entity_name.strip().casefold()
    if not term:
        return []
    return [node for node in graph.nodes() if node.id.casefold() == term or node.name.casefold() == term]


def resolve_ids(graph: SemanticGraph, entity: str) -> list[str]:
    return [node.id for node in query_entities(graph, entity)]

