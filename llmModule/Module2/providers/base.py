"""Replaceable semantic-map access contract."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Mapping

from semantic_graph import SemanticGraph, SemanticNode


class SemanticMapProvider(ABC):
    @property
    @abstractmethod
    def graph(self) -> SemanticGraph:
        raise NotImplementedError

    @property
    @abstractmethod
    def revision(self) -> str:
        raise NotImplementedError

    @property
    @abstractmethod
    def ontology(self) -> Mapping[str, tuple[str, ...]]:
        raise NotImplementedError

    @abstractmethod
    def instances(self, canonical_name: str) -> list[SemanticNode]:
        raise NotImplementedError

    @abstractmethod
    def entity(self, entity_id: str) -> SemanticNode | None:
        raise NotImplementedError

    @abstractmethod
    def navigation_pose(self, node: SemanticNode) -> tuple[float, float, float]:
        raise NotImplementedError

    @abstractmethod
    def topology_path(self, start: str, goal: str) -> list[str]:
        raise NotImplementedError
