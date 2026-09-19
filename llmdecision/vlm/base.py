"""Provider-neutral interface for image-to-scene-description inference."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path

from api.input_schema import SceneDescription


class VLMProviderError(RuntimeError):
    """A sanitized perception failure with a stable machine-readable code."""

    def __init__(self, code: str) -> None:
        self.code = code
        super().__init__(code)


@dataclass(frozen=True)
class VLMResult:
    provider: str
    latency_seconds: float
    scene_description: SceneDescription


class BaseVLM(ABC):
    """Minimal contract shared by remote and test VLM providers."""

    @abstractmethod
    def describe(self, image_path: Path, request_id: str) -> VLMResult:
        """Return one validated scene description for an image."""
