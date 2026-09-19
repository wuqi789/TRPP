"""Public target-detector contract and deterministic mock implementation.

The public workspace deliberately does not ship a detector model or inference
implementation.  A private deployment can provide an adapter through
``SCOUT_TARGET_DETECTOR_ADAPTER=module:factory``.  That adapter owns model
selection, prompts, weights, hardware, transport and preprocessing details.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import importlib
import os
import re
from typing import Any, Mapping

import numpy as np

from .core import Candidate


_FACTORY_SPEC = re.compile(r"^[A-Za-z_][A-Za-z0-9_.]*:[A-Za-z_][A-Za-z0-9_]*$")


class DetectorError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)


class BaseDetector(ABC):
    @abstractmethod
    def detect(self, image_bgr: np.ndarray, caption: str) -> list[Candidate]:
        """Return pixel-space candidates for one BGR image."""


class MockDetector(BaseDetector):
    """Deterministic curtain-like candidate used by the public demo."""

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.detected = bool(config.get("detected", True))
        self.confidence = float(config.get("confidence", 0.95))
        if not 0.0 <= self.confidence <= 1.0:
            raise DetectorError("MOCK_CONFIG_INVALID", "mock confidence must be in [0, 1]")
        bbox = config.get("bbox_normalized", (0.25, 0.25, 0.75, 0.75))
        try:
            self.bbox_normalized = tuple(float(value) for value in bbox)
        except (TypeError, ValueError) as exc:
            raise DetectorError(
                "MOCK_CONFIG_INVALID",
                "mock bbox_normalized must contain four numbers",
            ) from exc
        if (
            len(self.bbox_normalized) != 4
            or min(self.bbox_normalized) < 0.0
            or max(self.bbox_normalized) > 1.0
            or self.bbox_normalized[0] >= self.bbox_normalized[2]
            or self.bbox_normalized[1] >= self.bbox_normalized[3]
        ):
            raise DetectorError(
                "MOCK_CONFIG_INVALID",
                "mock bbox_normalized must be a valid normalized xyxy rectangle",
            )

    def detect(self, image_bgr: np.ndarray, caption: str) -> list[Candidate]:
        if not self.detected or not caption:
            return []
        height, width = image_bgr.shape[:2]
        if width < 4 or height < 4:
            raise DetectorError("IMAGE_TOO_SMALL", "input image must be at least 4x4")
        x_min = max(0, min(width - 1, int(round(self.bbox_normalized[0] * width))))
        y_min = max(0, min(height - 1, int(round(self.bbox_normalized[1] * height))))
        x_max = max(x_min + 1, min(width, int(round(self.bbox_normalized[2] * width))))
        y_max = max(y_min + 1, min(height, int(round(self.bbox_normalized[3] * height))))
        return [
            Candidate(
                phrase=caption.rstrip("."),
                confidence=self.confidence,
                bbox=(x_min, y_min, x_max, y_max),
            )
        ]


class ExternalDetector(BaseDetector):
    """Delegate detection to an installed private adapter.

    The adapter receives the image and a versioned, provider-neutral request.
    It must expose ``detect(image_bgr, request)`` and return either a sequence
    of ``Candidate`` objects or mappings with ``phrase``, ``confidence`` and
    ``bbox`` fields.  Provider prompts, model paths, credentials and inference
    code remain outside this repository.
    """

    def __init__(self, config: Mapping[str, Any]) -> None:
        factory_env = str(
            config.get("factory_env", "SCOUT_TARGET_DETECTOR_ADAPTER")
        ).strip()
        spec = os.getenv(factory_env, "").strip()
        if not spec:
            raise DetectorError("DETECTOR_ADAPTER_NOT_CONFIGURED", "adapter is not configured")
        if not _FACTORY_SPEC.fullmatch(spec):
            raise DetectorError("DETECTOR_ADAPTER_INVALID", "adapter spec must use module:factory")
        module_name, factory_name = spec.split(":", 1)
        try:
            factory = getattr(importlib.import_module(module_name), factory_name)
            self._adapter = factory()
        except Exception:
            raise DetectorError(
                "DETECTOR_ADAPTER_INITIALIZATION_FAILED",
                "adapter initialization failed",
            ) from None
        if not callable(getattr(self._adapter, "detect", None)):
            raise DetectorError("DETECTOR_ADAPTER_INVALID", "adapter must provide callable detect")

    def detect(self, image_bgr: np.ndarray, caption: str) -> list[Candidate]:
        request = {
            "contract": "scout.target-detection.v1",
            "target_label": str(caption),
            "response_schema": {
                "candidates": [
                    {"phrase": "string", "confidence": "number", "bbox": ["x1", "y1", "x2", "y2"]}
                ]
            },
        }
        try:
            response = self._adapter.detect(image_bgr, request)
        except Exception:
            raise DetectorError(
                "DETECTOR_ADAPTER_FAILED",
                "adapter invocation failed",
            ) from None
        if isinstance(response, Mapping):
            response = response.get("candidates")
        if response is None or isinstance(response, (str, bytes)):
            raise DetectorError("DETECTOR_RESPONSE_INVALID", "adapter returned no candidate sequence")
        try:
            values = list(response)
        except TypeError:
            raise DetectorError("DETECTOR_RESPONSE_INVALID", "adapter returned no candidate sequence") from None
        output: list[Candidate] = []
        for value in values:
            if isinstance(value, Candidate):
                output.append(value)
                continue
            if not isinstance(value, Mapping):
                raise DetectorError("DETECTOR_RESPONSE_INVALID", "candidate must be a mapping")
            try:
                phrase = str(value["phrase"]).strip() or "target"
                confidence = float(value["confidence"])
                bbox_values = tuple(int(round(float(item))) for item in value["bbox"])
                if len(bbox_values) != 4:
                    raise ValueError("bbox must contain four values")
                output.append(Candidate(phrase, confidence, bbox_values))
            except (KeyError, TypeError, ValueError) as exc:
                raise DetectorError("DETECTOR_RESPONSE_INVALID", "invalid candidate fields") from exc
        return output


def create_detector(config: Mapping[str, Any]) -> BaseDetector:
    provider = str(config.get("provider", "mock")).strip().lower()
    if provider == "mock":
        return MockDetector(config)
    if provider == "external":
        return ExternalDetector(config)
    raise DetectorError("DETECTOR_PROVIDER_INVALID", "provider must be mock or external")
