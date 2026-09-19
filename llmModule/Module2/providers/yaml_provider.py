"""YAML + NetworkX implementation of the semantic-map contract."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any, Mapping

from builder import build_graph, load_map
from query.coordinates import relative_pose
from query.topology_query import query_topology
from semantic_graph import SemanticGraph, SemanticNode

from .base import SemanticMapProvider


class YamlSemanticMapProvider(SemanticMapProvider):
    def __init__(self, path: str | Path, navigation_origin_id: str = "jackal_robot") -> None:
        self.path = Path(path)
        raw = self.path.read_bytes()
        data = load_map(self.path)
        self._graph = build_graph(data)
        self._revision = hashlib.sha256(raw).hexdigest()
        self._origin = self._graph.get_node(navigation_origin_id) if navigation_origin_id else None
        if navigation_origin_id and self._origin is None:
            raise ValueError(f"navigation origin entity does not exist: {navigation_origin_id}")
        self._ontology = self._load_ontology(data)

    @property
    def graph(self) -> SemanticGraph:
        return self._graph

    @property
    def revision(self) -> str:
        return self._revision

    @property
    def ontology(self) -> Mapping[str, tuple[str, ...]]:
        return self._ontology

    def instances(self, canonical_name: str) -> list[SemanticNode]:
        term = canonical_name.casefold()
        return [node for node in self._graph.nodes() if node.name.casefold() == term]

    def entity(self, entity_id: str) -> SemanticNode | None:
        return self._graph.get_node(entity_id)

    def navigation_pose(self, node: SemanticNode) -> tuple[float, float, float]:
        if self._origin is None:
            return node.position.x, node.position.y, node.position.theta
        return relative_pose(node.position, self._origin.position)

    def topology_path(self, start: str, goal: str) -> list[str]:
        return query_topology(self._graph, start, goal)

    @staticmethod
    def _load_ontology(data: Mapping[str, Any]) -> dict[str, tuple[str, ...]]:
        configured = data.get("ontology", {})
        result: dict[str, tuple[str, ...]] = {}
        if configured and not isinstance(configured, Mapping):
            raise ValueError("map section 'ontology' must be a mapping")
        for canonical, entry in configured.items():
            if isinstance(entry, Mapping):
                aliases = entry.get("aliases", [])
            else:
                aliases = entry
            if not isinstance(aliases, list) or not all(isinstance(value, str) for value in aliases):
                raise ValueError(f"ontology aliases for '{canonical}' must be a string list")
            result[str(canonical)] = tuple(str(value) for value in aliases)

        for section in ("rooms", "objects", "waypoints", "doors", "corridors"):
            for entry in data.get(section, []):
                name = str(entry.get("name", entry.get("id", ""))).strip()
                if name:
                    result.setdefault(name, ())
        return result
