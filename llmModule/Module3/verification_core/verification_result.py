"""Verification result value object."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class VerificationResult:
    verified: bool
    goal_id: str = ""
    goal_pose: dict[str, float] = field(default_factory=dict)
    failed_checks: tuple[str, ...] = ()
    explanation: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "verified": self.verified,
            "goal_id": self.goal_id,
            "goal_pose": dict(self.goal_pose),
            "failed_checks": list(self.failed_checks),
            "explanation": self.explanation,
        }
