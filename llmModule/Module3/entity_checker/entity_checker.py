"""Verify that a semantic target exists in the graph."""

from __future__ import annotations

from typing import Any


class EntityChecker:
    """Exact string matcher; embedding similarity can be added behind this interface."""

    def check(self, entity_name: str, semantic_graph: Any) -> dict[str, Any]:
        term = entity_name.strip().casefold()
        if not term:
            return {"success": False, "candidate_id": ""}
        for node_id, name in self._entities(semantic_graph):
            if term in {node_id.casefold(), name.casefold()}:
                return {"success": True, "candidate_id": node_id}
        return {"success": False, "candidate_id": ""}

    @staticmethod
    def _entities(graph: Any):
        if hasattr(graph, "nodes") and callable(graph.nodes):
            nodes = graph.nodes()
            if isinstance(nodes, list):
                for node in nodes:
                    yield str(node.id), str(getattr(node, "name", node.id))
                return
        nx_graph = getattr(graph, "networkx_graph", graph)
        for node_id, attributes in nx_graph.nodes(data=True):
            entity = attributes.get("entity")
            name = getattr(entity, "name", attributes.get("name", node_id))
            yield str(node_id), str(name)

