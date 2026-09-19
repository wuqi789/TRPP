"""Compatibility boundary for deployment-owned confidence policy.

The public workspace does not include confidence weighting, calibration, or
uncertainty rules. Those rules belong to the separately installed decision
adapter. Basic probability normalization remains available for wire-schema
compatibility; policy calculations fail closed.
"""

from __future__ import annotations

import math
from typing import Mapping


def clamp_probability(value: float) -> float:
    """Return a finite probability in the closed interval [0, 1]."""

    numeric = float(value)
    if not math.isfinite(numeric):
        return 0.0
    return min(1.0, max(0.0, numeric))


def normalize_distribution(values: Mapping[str, float]) -> dict[str, float]:
    """Clamp and normalize push/avoid/stop values with a safe zero fallback."""

    normalized = {
        key: clamp_probability(values.get(key, 0.0))
        for key in ("push", "avoid", "stop")
    }
    total = sum(normalized.values())
    if total <= 0.0:
        return {"push": 0.0, "avoid": 0.0, "stop": 1.0}
    return {key: value / total for key, value in normalized.items()}


def calculate_evidence_confidence(
    signals: Mapping[str, float],
    weights: Mapping[str, float],
) -> float:
    """Reject local policy evaluation; use the external adapter instead."""

    del signals, weights
    raise RuntimeError("external confidence policy is required")


def calculate_uncertainty(llm_uncertainty: float, evidence_confidence: float) -> float:
    """Reject local policy evaluation; use the external adapter instead."""

    del llm_uncertainty, evidence_confidence
    raise RuntimeError("external uncertainty policy is required")
