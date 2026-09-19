"""Supported semantic edge relations."""

from enum import Enum


class Relation(str, Enum):
    CONNECTED = "connected"
    INSIDE = "inside"
    CONTAINS = "contains"
    NEAR = "near"
    FAR = "far"
    LEFT_OF = "left_of"
    RIGHT_OF = "right_of"
    BEHIND = "behind"
    FRONT_OF = "front_of"
    REACHABLE = "reachable"

    @classmethod
    def parse(cls, value: str) -> "Relation":
        try:
            return cls(value.strip().lower())
        except ValueError as exc:
            raise ValueError(f"unsupported relation: {value}") from exc


# Containment attaches entities to navigable spaces. This lets two entities in
# the same room be semantically reachable while CONNECTED/REACHABLE still join
# rooms, corridors, and waypoints.
TOPOLOGY_RELATIONS = frozenset(
    {Relation.CONNECTED, Relation.REACHABLE, Relation.INSIDE, Relation.CONTAINS}
)
