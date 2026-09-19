"""Deterministic offline LLM used for local development and tests."""

import json

from llm.base import BaseLLM


class MockLLM(BaseLLM):
    """Return a fixed, schema-compliant assessment without network access."""

    def generate(self, request: str) -> str:
        if not isinstance(request, str) or not request.strip():
            raise ValueError("request must be a non-empty string")
        response = {
            "object_assessment": {
                "class": "mock_obstacle",
                "confidence": 0.5,
            },
            "pushability_probability": 0.5,
            "decision_distribution": {
                "push": 0.5,
                "avoid": 0.3,
                "stop": 0.2,
            },
            "risk_flags": ["mock_result"],
            "uncertainty": 0.5,
            "reason": "Deterministic public mock response; no physical inference was run.",
        }
        return json.dumps(response, ensure_ascii=False)
