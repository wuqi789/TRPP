"""Node types used by the semantic graph."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


SUPPORTED_NODE_TYPES = frozenset({"room", "object", "waypoint", "door", "corridor"})


@dataclass(frozen=True)
class Position:
    x: float
    y: float
    theta: float = 0.0

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "Position":
        try:
            return cls(float(value["x"]), float(value["y"]), float(value.get("theta", 0.0)))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("position requires numeric x and y values") from exc

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "theta": self.theta}


@dataclass(frozen=True)
class SemanticNode:
    id: str
    type: str
    name: str
    position: Position
    properties: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("node id must not be empty")
        if self.type not in SUPPORTED_NODE_TYPES:
            raise ValueError(f"unsupported node type: {self.type}")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "SemanticNode":
        node_id = str(value.get("id", "")).strip()
        return cls(
            id=node_id,
            type=str(value.get("type", "")).strip(),
            name=str(value.get("name", node_id)).strip(),
            position=Position.from_mapping(value.get("position", {})),
            properties=dict(value.get("properties", {})),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "type": self.type,
            "name": self.name,
            "position": self.position.to_dict(),
            "properties": dict(self.properties),
        }

