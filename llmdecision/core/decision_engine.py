"""Provider-neutral orchestration of retrieval, parsing, and fusion."""

from __future__ import annotations

import json
import logging
import math
import re
from dataclasses import replace
from typing import Any, Mapping

from api.input_schema import DecisionInput
from api.output_schema import (
    DECISION_JSON_SCHEMA,
    DecisionDistribution,
    DecisionOutput,
    ObjectAssessment,
)
from core.fusion import FusionConfig, fuse_decision
from core.prompt_builder import PromptBuilder
from llm.base import BaseLLM
from rag.retriever import BaseRetriever

LOGGER = logging.getLogger(__name__)
_PUBLIC_FLAG = re.compile(r"^[a-z0-9_.-]{1,64}$")


class DecisionParseError(ValueError):
    """Raised when provider output violates the required JSON schema."""


def _object(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise DecisionParseError(f"{path} must be a JSON object")
    return value


def _field(data: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in data:
        raise DecisionParseError(f"{path}.{key} is required")
    return data[key]


def _schema_object(
    value: Any,
    path: str,
    schema: Mapping[str, Any],
) -> Mapping[str, Any]:
    data = _object(value, path)
    properties = set(schema["properties"])
    required = set(schema["required"])
    missing = sorted(required - set(data))
    if missing:
        raise DecisionParseError(f"{path}.{missing[0]} is required")
    extra = sorted(set(data) - properties)
    if extra:
        raise DecisionParseError(f"{path}.{extra[0]} is not allowed")
    return data


def _probability(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DecisionParseError(f"{path} must be a number in [0, 1]")
    result = float(value)
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise DecisionParseError(f"{path} must be a number in [0, 1]")
    return result


def _non_empty_string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise DecisionParseError(f"{path} must be a non-empty string")
    return value.strip()


def _public_object_class(value: Any, path: str) -> str:
    result = _non_empty_string(value, path)
    if len(result) > 64 or any(ord(character) < 32 for character in result):
        return "provider_object_redacted"
    if re.search(
        r"(?i)(?:system\s+prompt|chain\s*[- ]?of\s*[- ]?thought|few\s*[- ]?shot|"
        r"api[_ -]?key|authorization|bearer|ssh|endpoint|model\s+path|internal\s+rule)",
        result,
    ):
        return "provider_object_redacted"
    return result


def _parse_llm_response(response: str) -> DecisionOutput:
    if not isinstance(response, str):
        raise DecisionParseError("LLM response must be a string")
    try:
        document = json.loads(response)
    except json.JSONDecodeError:
        raise DecisionParseError("LLM response is not valid JSON") from None
    root = _schema_object(document, "response", DECISION_JSON_SCHEMA)
    assessment_schema = DECISION_JSON_SCHEMA["properties"]["object_assessment"]
    assessment = _schema_object(
        _field(root, "object_assessment", "response"),
        "response.object_assessment",
        assessment_schema,
    )
    distribution_schema = DECISION_JSON_SCHEMA["properties"][
        "decision_distribution"
    ]
    distribution = _schema_object(
        _field(root, "decision_distribution", "response"),
        "response.decision_distribution",
        distribution_schema,
    )
    risk_flags_value = _field(root, "risk_flags", "response")
    if not isinstance(risk_flags_value, list) or any(
        not isinstance(flag, str) or not flag.strip() for flag in risk_flags_value
    ):
        raise DecisionParseError("response.risk_flags must be an array of strings")
    normalized_flags = [
        flag.strip() if _PUBLIC_FLAG.fullmatch(flag.strip()) else "provider_flag_redacted"
        for flag in risk_flags_value
    ]
    if len(normalized_flags) != len(set(normalized_flags)):
        raise DecisionParseError("response.risk_flags must contain unique values")

    push = _probability(
        _field(distribution, "push", "response.decision_distribution"),
        "response.decision_distribution.push",
    )
    avoid = _probability(
        _field(distribution, "avoid", "response.decision_distribution"),
        "response.decision_distribution.avoid",
    )
    stop = _probability(
        _field(distribution, "stop", "response.decision_distribution"),
        "response.decision_distribution.stop",
    )
    if not math.isclose(push + avoid + stop, 1.0, abs_tol=1e-6):
        raise DecisionParseError(
            "response.decision_distribution probabilities must sum to 1"
        )

    return DecisionOutput(
        object_assessment=ObjectAssessment(
            object_class=_public_object_class(
                _field(assessment, "class", "response.object_assessment"),
                "response.object_assessment.class",
            ),
            confidence=_probability(
                _field(assessment, "confidence", "response.object_assessment"),
                "response.object_assessment.confidence",
            ),
        ),
        pushability_probability=_probability(
            _field(root, "pushability_probability", "response"),
            "response.pushability_probability",
        ),
        decision_distribution=DecisionDistribution(
            push=push,
            avoid=avoid,
            stop=stop,
        ),
        risk_flags=tuple(normalized_flags),
        uncertainty=_probability(
            _field(root, "uncertainty", "response"),
            "response.uncertainty",
        ),
            # Provider rationale may contain prompts, chain-of-thought, or
            # deployment policy. Keep the field for wire compatibility but do
            # not persist or publish its contents.
            reason="provider_reason_redacted",
    )


class DecisionEngine:
    """Provider-neutral obstacle pushability reasoning pipeline."""

    def __init__(
        self,
        llm: BaseLLM,
        retriever: BaseRetriever,
        prompt_builder: PromptBuilder,
        fusion_config: FusionConfig,
        top_k: int = 3,
    ) -> None:
        if isinstance(top_k, bool) or not isinstance(top_k, int) or top_k <= 0:
            raise ValueError("top_k must be a positive integer")
        self._llm = llm
        self._retriever = retriever
        self._prompt_builder = prompt_builder
        self._fusion_config = fusion_config
        self._top_k = top_k

    def decide(self, payload: Mapping[str, Any] | DecisionInput) -> DecisionOutput:
        """Validate input and return one fully structured decision."""

        decision_input = (
            payload if isinstance(payload, DecisionInput) else DecisionInput.from_dict(payload)
        )
        pipeline_risks: list[str] = []
        try:
            retrieved_cases = self._retriever.retrieve(
                decision_input.scene_description.text,
                top_k=self._top_k,
            )
        except Exception:
            LOGGER.error("RAG retrieval failed; continuing without retrieved cases")
            retrieved_cases = []
            pipeline_risks.append("rag_retrieval_failed")

        request = self._prompt_builder.build(decision_input, retrieved_cases)
        try:
            response = self._llm.generate(request)
        except Exception:
            LOGGER.error("Decision provider failed; returning contract fallback")
            return self._fallback(
                decision_input,
                "llm_provider_failed",
                "provider_unavailable",
                pipeline_risks,
            )

        try:
            raw_decision = _parse_llm_response(response)
        except DecisionParseError:
            LOGGER.error("LLM returned invalid structured output")
            return self._fallback(
                decision_input,
                "llm_output_invalid",
                "structured_output_unavailable",
                pipeline_risks,
            )

        if pipeline_risks:
            raw_decision = replace(
                raw_decision,
                risk_flags=raw_decision.risk_flags + tuple(pipeline_risks),
            )
        return fuse_decision(
            raw_decision,
            decision_input,
            retrieved_cases,
            self._fusion_config,
        )

    @staticmethod
    def _fallback(
        decision_input: DecisionInput,
        primary_risk: str,
        reason: str,
        additional_risks: list[str],
    ) -> DecisionOutput:
        risks = [primary_risk, *additional_risks]
        return DecisionOutput(
            object_assessment=ObjectAssessment(
                object_class="unknown_obstacle",
                confidence=0.0,
            ),
            pushability_probability=0.0,
            decision_distribution=DecisionDistribution(
                push=0.0,
                avoid=0.25,
                stop=0.75,
            ),
            risk_flags=tuple(dict.fromkeys(risks)),
            uncertainty=1.0,
            reason=reason,
        )
