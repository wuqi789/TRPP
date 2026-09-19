"""Semantic graph edge model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .relation import Relation


@dataclass(frozen=True)
class SemanticEdge:
    source: str
    target: str
    relation: Relation
    distance: float = 0.0
    confidence: float = 1.0

    def __post_init__(self) -> None:
        if not self.source or not self.target:
            raise ValueError("edge source and target must not be empty")
        if self.distance < 0:
            raise ValueError("edge distance must be non-negative")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("edge confidence must be between 0 and 1")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "SemanticEdge":
        return cls(
            source=str(value.get("source", "")).strip(),
            target=str(value.get("target", "")).strip(),
            relation=Relation.parse(str(value.get("relation", ""))),
            distance=float(value.get("distance", 0.0)),
            confidence=float(value.get("confidence", 1.0)),
        )

