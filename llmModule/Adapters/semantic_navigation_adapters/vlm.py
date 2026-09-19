"""Public VLM adapter boundary.

Private adapters own prompts, model selection, credentials, network transport and
retry policy.  This module deliberately knows only a versioned request contract.
"""

from __future__ import annotations

import copy
import importlib
import json
import os
import re
from typing import Any, Mapping


_ADAPTER_SPEC = re.compile(
    r"^[A-Za-z_][A-Za-z0-9_.]*:[A-Za-z_][A-Za-z0-9_]*$"
)


class VLMError(RuntimeError):
    """The adapter returned data that violates the public contract."""


class VLMConfigurationError(VLMError):
    """The external adapter is missing or does not implement the contract."""


class VLMTransportError(VLMError):
    """The private adapter reported a retryable provider failure."""


def _load_adapter(factory_env: str) -> Any:
    spec = os.getenv(factory_env, "").strip()
    if not spec:
        raise VLMConfigurationError("external adapter is not configured")
    if not _ADAPTER_SPEC.fullmatch(spec):
        raise VLMConfigurationError("external adapter spec must use module:factory")
    module_name, factory_name = spec.split(":", 1)
    try:
        factory = getattr(importlib.import_module(module_name), factory_name)
        adapter = factory()
    except Exception:
        raise VLMConfigurationError("external adapter initialization failed") from None
    if not callable(getattr(adapter, "infer", None)):
        raise VLMConfigurationError(
            "external adapter must provide callable infer(image, request)"
        )
    return adapter


class ExternalVLMAdapter:
    """Invoke a separately installed adapter through a narrow data-only API."""

    provider_kind = "external"
    last_transport = "not_called"

    def __init__(self, factory_env: str) -> None:
        self.factory_env = factory_env
        self._adapter = _load_adapter(factory_env)

    @property
    def ready(self) -> bool:
        return True

    def infer(
        self,
        image: bytes,
        request: Mapping[str, Any],
        feedback: Mapping[str, Any] | None = None,
        *,
        mime_type: str = "image/png",
    ) -> str:
        envelope = copy.deepcopy(dict(request))
        envelope["image"] = {"mime_type": mime_type}
        if feedback:
            envelope["validation_feedback"] = copy.deepcopy(dict(feedback))
        try:
            response = self._adapter.infer(bytes(image), envelope)
        except Exception as exc:
            # A private adapter may mark transient failures without exposing its
            # provider-specific exception hierarchy to the public repository.
            if bool(getattr(exc, "retryable", False)):
                raise VLMTransportError("external adapter request failed") from None
            raise VLMError("external adapter request failed") from None
        self.last_transport = "external_adapter"
        if isinstance(response, Mapping):
            return json.dumps(dict(response), separators=(",", ":"))
        if isinstance(response, str):
            return response
        raise VLMError("external adapter returned an unsupported response type")


class MockVLMAdapter:
    """Deterministic local fixture for the public demonstration."""

    provider_kind = "mock"
    last_transport = "local_mock"
    ready = True

    def infer(
        self,
        image: bytes,
        request: Mapping[str, Any],
        feedback: Mapping[str, Any] | None = None,
        *,
        mime_type: str = "image/png",
    ) -> str:
        del image, feedback, mime_type
        contract = str(request.get("contract", ""))
        if contract == "scout.route-waypoints.v1":
            return '{"waypoints":[]}'
        if contract == "scout.pushability-map.v1":
            return '{"objects":[]}'
        raise VLMError("mock adapter does not support the requested contract")


def create_vlm_adapter(mode: str, factory_env: str):
    normalized = mode.strip().lower()
    if normalized == "mock":
        return MockVLMAdapter()
    if normalized == "external":
        return ExternalVLMAdapter(factory_env)
    raise VLMConfigurationError("VLM mode must be mock or external")


__all__ = [
    "ExternalVLMAdapter",
    "MockVLMAdapter",
    "VLMConfigurationError",
    "VLMError",
    "VLMTransportError",
    "create_vlm_adapter",
]
