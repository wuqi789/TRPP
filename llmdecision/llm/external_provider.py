"""Decision provider boundary backed by a separately installed private adapter."""

from __future__ import annotations

import json
from typing import Any, Mapping

from llm.base import BaseLLM, LLMProviderError
from providers.loader import ExternalAdapterError, load_external_adapter


class ExternalLLMProvider(BaseLLM):
    """Forward a versioned JSON request to an out-of-repository adapter."""

    model = "external-adapter"

    def __init__(self, factory_env: str = "SCOUT_DECISION_ADAPTER") -> None:
        self.factory_env = str(factory_env).strip()
        if not self.factory_env:
            raise ValueError("factory_env must be a non-empty string")
        try:
            self._adapter = load_external_adapter(self.factory_env, "assess")
        except ExternalAdapterError:
            raise LLMProviderError("external_adapter_unavailable") from None

    def generate(self, request_json: str) -> str:
        try:
            request = json.loads(request_json)
            if not isinstance(request, Mapping):
                raise ValueError
            response: Any = self._adapter.assess(dict(request))
            if isinstance(response, Mapping):
                return json.dumps(dict(response), ensure_ascii=False)
            if isinstance(response, str) and response.strip():
                return response
        except LLMProviderError:
            raise
        except Exception:
            raise LLMProviderError("external_adapter_request_failed") from None
        raise LLMProviderError("external decision adapter returned an invalid response")
