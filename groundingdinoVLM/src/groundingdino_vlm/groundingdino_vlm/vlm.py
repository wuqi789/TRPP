"""Public target-verification contracts and deterministic mock provider."""

from __future__ import annotations

from abc import ABC, abstractmethod
import importlib
import os
import re
from typing import Any, Mapping, Sequence

import numpy as np

from .core import Candidate


_FACTORY_SPEC = re.compile(r"^[A-Za-z_][A-Za-z0-9_.]*:[A-Za-z_][A-Za-z0-9_]*$")


class VLMError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)


VLMConfigurationError = VLMError


class BaseVLM(ABC):
    provider: str
    model: str

    @abstractmethod
    def verify(
        self,
        image_bgr: np.ndarray,
        target_label: str,
        candidates: Sequence[Candidate],
    ) -> bool:
        """Return whether at least one candidate matches the public target label."""


def parse_vlm_decision(value: str) -> bool:
    decision = str(value).strip()
    if decision == "ACCEPT":
        return True
    if decision == "REJECT":
        return False
    raise VLMError("VLM_RESPONSE_INVALID", "adapter decision must be ACCEPT or REJECT")


class MockVLM(BaseVLM):
    def __init__(self, config: Mapping[str, Any]) -> None:
        self.provider = "mock"
        self.model = "local-mock"
        self.decision = str(config.get("decision") or "ACCEPT").strip()
        self.error_code = str(config.get("error_code") or "").strip()

    def verify(
        self,
        image_bgr: np.ndarray,
        target_label: str,
        candidates: Sequence[Candidate],
    ) -> bool:
        del image_bgr, target_label, candidates
        if self.error_code:
            raise VLMError(self.error_code, "configured mock VLM failure")
        return parse_vlm_decision(self.decision)


class ExternalVLM(BaseVLM):
    """Delegate identity verification to an installed private adapter.

    The adapter owns all prompts, credentials, model choices and network or local
    inference. Its factory is selected as ``module:factory`` through ``factory_env``.
    """

    provider = "external-adapter"
    model = "external-adapter"

    def __init__(self, config: Mapping[str, Any]) -> None:
        factory_env = str(config.get("factory_env", "SCOUT_TARGET_VLM_ADAPTER")).strip()
        spec = os.getenv(factory_env, "").strip()
        if not spec:
            raise VLMError("VLM_ADAPTER_NOT_CONFIGURED", "adapter is not configured")
        if not _FACTORY_SPEC.fullmatch(spec):
            raise VLMError("VLM_ADAPTER_INVALID", "adapter spec must use module:factory")
        module_name, factory_name = spec.split(":", 1)
        try:
            factory = getattr(importlib.import_module(module_name), factory_name)
            self._adapter = factory()
        except Exception:
            raise VLMError(
                "VLM_ADAPTER_INITIALIZATION_FAILED",
                "adapter initialization failed",
            ) from None
        if not callable(getattr(self._adapter, "verify", None)):
            raise VLMError("VLM_ADAPTER_INVALID", "adapter must provide callable verify")

    def verify(
        self,
        image_bgr: np.ndarray,
        target_label: str,
        candidates: Sequence[Candidate],
    ) -> bool:
        request = {
            "contract": "scout.target-verification.v1",
            "target_label": str(target_label),
            "candidates": [
                {
                    "phrase": item.phrase,
                    "confidence": float(item.confidence),
                    "bbox": list(item.bbox),
                }
                for item in candidates
            ],
        }
        try:
            response = self._adapter.verify(image_bgr, request)
        except Exception:
            raise VLMError(
                "VLM_ADAPTER_FAILED",
                "adapter invocation failed",
            ) from None
        if isinstance(response, bool):
            return response
        if isinstance(response, str):
            return parse_vlm_decision(response)
        if isinstance(response, Mapping) and isinstance(response.get("accepted"), bool):
            return bool(response["accepted"])
        raise VLMError("VLM_RESPONSE_INVALID", "adapter returned an invalid response")


def create_vlm(config: Mapping[str, Any]) -> BaseVLM:
    provider = str(config.get("provider", "external")).strip().lower()
    if provider == "mock":
        return MockVLM(config)
    if provider == "external":
        return ExternalVLM(config)
    raise VLMError("VLM_PROVIDER_INVALID", "provider must be mock or external")
