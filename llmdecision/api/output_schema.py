"""Structured result types for obstacle pushability reasoning."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


DECISION_JSON_SCHEMA: dict[str, Any] = {
    "type": "object",
    "additionalProperties": False,
    "required": [
        "object_assessment",
        "pushability_probability",
        "decision_distribution",
        "risk_flags",
        "uncertainty",
        "reason",
    ],
    "properties": {
        "object_assessment": {
            "type": "object",
            "additionalProperties": False,
            "required": ["class", "confidence"],
            "properties": {
                "class": {"type": "string", "minLength": 1},
                "confidence": {"type": "number", "minimum": 0, "maximum": 1},
            },
        },
        "pushability_probability": {
            "type": "number",
            "minimum": 0,
            "maximum": 1,
        },
        "decision_distribution": {
            "type": "object",
            "additionalProperties": False,
            "required": ["push", "avoid", "stop"],
            "properties": {
                "push": {"type": "number", "minimum": 0, "maximum": 1},
                "avoid": {"type": "number", "minimum": 0, "maximum": 1},
                "stop": {"type": "number", "minimum": 0, "maximum": 1},
            },
        },
        "risk_flags": {
            "type": "array",
            "items": {"type": "string", "minLength": 1},
            "uniqueItems": True,
        },
        "uncertainty": {"type": "number", "minimum": 0, "maximum": 1},
        "reason": {"type": "string", "minLength": 1},
    },
}


def _clamp(value: float) -> float:
    numeric = float(value)
    if not math.isfinite(numeric):
        return 0.0
    return min(1.0, max(0.0, numeric))


@dataclass(frozen=True)
class ObjectAssessment:
    object_class: str
    confidence: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "class": self.object_class,
            "confidence": round(_clamp(self.confidence), 6),
        }


@dataclass(frozen=True)
class DecisionDistribution:
    push: float
    avoid: float
    stop: float

    def normalized(self) -> "DecisionDistribution":
        values = [_clamp(self.push), _clamp(self.avoid), _clamp(self.stop)]
        total = sum(values)
        if total <= 0.0:
            return DecisionDistribution(push=0.0, avoid=0.0, stop=1.0)
        return DecisionDistribution(*(value / total for value in values))

    def to_dict(self) -> dict[str, float]:
        normalized = self.normalized()
        push = round(normalized.push, 6)
        avoid = round(normalized.avoid, 6)
        stop = round(1.0 - push - avoid, 6)
        return {"push": push, "avoid": avoid, "stop": stop}


@dataclass(frozen=True)
class DecisionOutput:
    object_assessment: ObjectAssessment
    pushability_probability: float
    decision_distribution: DecisionDistribution
    risk_flags: tuple[str, ...]
    uncertainty: float
    reason: str

    def to_dict(self) -> dict[str, Any]:
        unique_flags = tuple(dict.fromkeys(flag for flag in self.risk_flags if flag))
        return {
            "object_assessment": self.object_assessment.to_dict(),
            "pushability_probability": round(
                _clamp(self.pushability_probability),
                6,
            ),
            "decision_distribution": self.decision_distribution.to_dict(),
            "risk_flags": list(unique_flags),
            "uncertainty": round(_clamp(self.uncertainty), 6),
            "reason": self.reason.strip(),
        }
