"""Verify graph connectivity without performing metric path planning."""

from __future__ import annotations

from typing import Any

import networkx as nx


class TopologyChecker:
    def check(self, start_node: str, goal_node: str, graph: Any) -> dict[str, Any]:
        nx_graph = graph.topology_view() if hasattr(graph, "topology_view") else graph
        try:
            path = nx.shortest_path(nx_graph, start_node, goal_node)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return {"reachable": False, "path": []}
        return {"reachable": True, "path": list(path)}
