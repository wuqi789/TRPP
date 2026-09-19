"""Public adapter contract for semantic navigation-intent extraction."""

from __future__ import annotations

import importlib
import json
import os
import re
from typing import Any, Mapping


_ADAPTER_SPEC = re.compile(
    r"^[A-Za-z_][A-Za-z0-9_.]*:[A-Za-z_][A-Za-z0-9_]*$"
)


class LLMError(RuntimeError):
    """Raised when the configured adapter cannot return a valid response."""

    def __init__(self, code: str, message: str | None = None) -> None:
        self.code = code
        super().__init__(message or code)


class LLMInterface:
    """Use the local fixture or a separately installed private adapter."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.mode = str(config.get("mode", "mock")).strip().lower()
        if self.mode not in {"mock", "external"}:
            raise ValueError("mode must be mock or external")
        self._adapter = None
        self.adapter_env = str(
            config.get("adapter_env", "SCOUT_NAVIGATION_INTENT_ADAPTER")
        )
        if self.mode == "external":
            self._adapter = self._load_adapter()

    @property
    def ready(self) -> bool:
        return self.mode == "mock" or self._adapter is not None

    def _load_adapter(self):
        spec = os.getenv(self.adapter_env, "").strip()
        if not spec:
            raise ValueError("external adapter is not configured")
        if not _ADAPTER_SPEC.fullmatch(spec):
            raise ValueError("external adapter spec must use module:factory")
        module_name, factory_name = spec.split(":", 1)
        try:
            factory = getattr(importlib.import_module(module_name), factory_name)
            adapter = factory()
        except Exception:
            raise ValueError("external adapter initialization failed") from None
        if not callable(getattr(adapter, "infer", None)):
            raise ValueError("external adapter contract is not implemented")
        return adapter

    def infer(self, user_text: str) -> str:
        instruction = user_text.strip()
        if not instruction:
            raise LLMError("EMPTY_INSTRUCTION", "Instruction must not be empty")
        if self.mode == "mock":
            return self._mock_response(instruction)
        request = {
            "contract": "scout.navigation-intent.v1",
            "instruction": instruction,
            "response_schema": {
                "goal_object": "string",
                "reference_object": "string",
                "relation": "string",
                "constraints": ["string"],
                "strategy": "string",
            },
        }
        try:
            response = self._adapter.infer(request)
        except Exception:
            raise LLMError(
                "EXTERNAL_ADAPTER_FAILED",
                "External adapter failed",
            ) from None
        if isinstance(response, Mapping):
            return json.dumps(dict(response), separators=(",", ":"))
        if isinstance(response, str):
            return response
        raise LLMError(
            "INVALID_PROVIDER_RESPONSE", "External adapter returned an unsupported type"
        )

    @staticmethod
    def _mock_response(text: str) -> str:
        """Return a schema fixture; language understanding belongs to an adapter."""
        del text
        return json.dumps({
            "goal_object": "demo_goal",
            "reference_object": "",
            "relation": "",
            "constraints": [],
            "strategy": "adapter_defined",
        })
