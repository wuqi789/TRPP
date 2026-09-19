"""NetworkX-backed semantic graph."""

from __future__ import annotations

import networkx as nx

from .edge import SemanticEdge
from .node import SemanticNode
from .relation import Relation, TOPOLOGY_RELATIONS


class SemanticGraph:
    def __init__(self) -> None:
        self._graph = nx.MultiDiGraph()

    @property
    def networkx_graph(self) -> nx.MultiDiGraph:
        return self._graph

    def add_node(self, node: SemanticNode) -> None:
        if node.id in self._graph:
            raise ValueError(f"duplicate node id: {node.id}")
        self._graph.add_node(node.id, entity=node, **node.to_dict())

    def add_edge(self, edge: SemanticEdge) -> None:
        if edge.source not in self._graph or edge.target not in self._graph:
            raise ValueError(f"edge references unknown node: {edge.source} -> {edge.target}")
        attributes = {
            "relation": edge.relation.value,
            "distance": edge.distance,
            "confidence": edge.confidence,
        }
        self._graph.add_edge(edge.source, edge.target, key=edge.relation.value, **attributes)
        if edge.relation == Relation.CONNECTED:
            self._graph.add_edge(edge.target, edge.source, key=edge.relation.value, **attributes)
        elif edge.relation == Relation.INSIDE:
            inverse = dict(attributes, relation=Relation.CONTAINS.value)
            self._graph.add_edge(edge.target, edge.source, key=Relation.CONTAINS.value, **inverse)
        elif edge.relation == Relation.CONTAINS:
            inverse = dict(attributes, relation=Relation.INSIDE.value)
            self._graph.add_edge(edge.target, edge.source, key=Relation.INSIDE.value, **inverse)

    def get_node(self, node_id: str) -> SemanticNode | None:
        if node_id not in self._graph:
            return None
        return self._graph.nodes[node_id]["entity"]

    def nodes(self) -> list[SemanticNode]:
        return [attributes["entity"] for _, attributes in self._graph.nodes(data=True)]

    def has_relation(self, source_id: str, relation: str, target_id: str) -> bool:
        edges = self._graph.get_edge_data(source_id, target_id, default={})
        return any(attributes.get("relation") == relation for attributes in edges.values())

    def topology_view(self) -> nx.DiGraph:
        topology = nx.DiGraph()
        topology.add_nodes_from(self._graph.nodes)
        allowed = {relation.value for relation in TOPOLOGY_RELATIONS}
        for source, target, attributes in self._graph.edges(data=True):
            if attributes.get("relation") in allowed:
                topology.add_edge(source, target, **attributes)
        return topology

