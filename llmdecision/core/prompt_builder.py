"""Serialize the public decision request contract."""

from __future__ import annotations

import json
from typing import Sequence

from api.input_schema import DecisionInput
from api.output_schema import DECISION_JSON_SCHEMA
from rag.retriever import RetrievalResult


class PromptBuilder:
    """Compatibility wrapper that builds a provider-neutral JSON envelope.

    Provider-specific instructions, policy calibration and model details belong
    to the separately distributed external adapter.
    """

    def build(
        self,
        decision_input: DecisionInput,
        retrieved_cases: Sequence[RetrievalResult],
    ) -> str:
        del retrieved_cases
        request = {
            "contract": "scout.pushability-assessment.v1",
            "input": decision_input.to_dict(),
            "response_schema": DECISION_JSON_SCHEMA,
        }
        return json.dumps(request, ensure_ascii=False, separators=(",", ":"))
