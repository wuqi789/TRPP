"""Public target-verification interfaces and mock providers."""

from .core import (
    Candidate,
    EngineOutput,
    TargetValidationEngine,
    ValidationSnapshot,
    ValidationState,
    normalize_target,
)

__all__ = [
    "Candidate",
    "EngineOutput",
    "TargetValidationEngine",
    "ValidationSnapshot",
    "ValidationState",
    "normalize_target",
]
