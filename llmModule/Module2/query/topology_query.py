"""Topological connectivity queries (not metric path planning)."""

import networkx as nx

from semantic_graph import SemanticGraph

from .entity_query import resolve_ids


def query_topology(graph: SemanticGraph, start: str, goal: str) -> list[str]:
    starts = resolve_ids(graph, start)
    goals = resolve_ids(graph, goal)
    topology = graph.topology_view()
    best: list[str] | None = None
    for start_id in starts:
        for goal_id in goals:
            try:
                candidate = nx.shortest_path(topology, start_id, goal_id)
            except (nx.NetworkXNoPath, nx.NodeNotFound):
                continue
            if best is None or len(candidate) < len(best):
                best = candidate
    return best or []

