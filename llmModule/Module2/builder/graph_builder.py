"""Build a SemanticGraph from a static map dictionary."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from semantic_graph import SemanticEdge, SemanticGraph, SemanticNode

from .yaml_loader import load_yaml


NODE_SECTIONS = ("rooms", "objects", "waypoints", "doors", "corridors")


def load_map(path: str | Path) -> dict[str, Any]:
    return load_yaml(path)


def build_graph(data: Mapping[str, Any]) -> SemanticGraph:
    graph = SemanticGraph()
    for section in NODE_SECTIONS:
        entries = data.get(section, [])
        if not isinstance(entries, list):
            raise ValueError(f"map section '{section}' must be a list")
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError(f"entries in '{section}' must be mappings")
            graph.add_node(SemanticNode.from_mapping(entry))
    relations = data.get("relations", [])
    if not isinstance(relations, list):
        raise ValueError("map section 'relations' must be a list")
    for entry in relations:
        if not isinstance(entry, dict):
            raise ValueError("relation entries must be mappings")
        graph.add_edge(SemanticEdge.from_mapping(entry))
    return graph

