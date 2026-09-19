"""Provider-neutral decision result boundary.

The public package validates and transports structured results.  Deployment
policy, confidence calibration and safety rules belong to the external adapter
and are deliberately absent from this repository.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from api.input_schema import DecisionInput
from api.output_schema import DecisionOutput
from rag.retriever import RetrievalResult


@dataclass(frozen=True)
class FusionConfig:
    """Opaque compatibility container for deployment-owned options."""

    options: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, value: Mapping[str, Any] | None) -> "FusionConfig":
        if value is None:
            return cls()
        if not isinstance(value, Mapping):
            raise ValueError("fusion config must be an object")
        # Keep configuration opaque so policy values are not embedded in the
        # public implementation.  External adapters may consume their own copy.
        return cls(dict(value))


def fuse_decision(
    raw_decision: DecisionOutput,
    decision_input: DecisionInput,
    retrieved_cases: Sequence[RetrievalResult],
    config: FusionConfig,
) -> DecisionOutput:
    """Return the validated provider result without applying hidden policy."""

    del decision_input, retrieved_cases, config
    return raw_decision
