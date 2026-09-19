"""Image perception boundary backed by a separately installed private adapter."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Mapping

from api.input_schema import SceneDescription, SceneEnvironment, SceneObstacle, SchemaValidationError
from providers.loader import ExternalAdapterError, load_external_adapter
from vlm.base import BaseVLM, VLMProviderError, VLMResult


def parse_scene_response(value: Any) -> SceneDescription:
    """Validate the public scene-description contract returned by an adapter."""

    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            raise VLMProviderError("vlm_output_invalid") from None
    if not isinstance(value, Mapping) or set(value) != {"environment", "obstacles"}:
        raise VLMProviderError("vlm_output_invalid")
    obstacles = value["obstacles"]
    if not isinstance(obstacles, list) or not obstacles or len(obstacles) > 5:
        raise VLMProviderError("vlm_output_invalid")
    try:
        environment = SceneEnvironment.from_dict(value["environment"], "response.environment")
        parsed = tuple(
            SceneObstacle.from_dict(item, f"response.obstacles[{index}]")
            for index, item in enumerate(obstacles)
        )
        return SceneDescription.from_perception(environment, parsed)
    except (SchemaValidationError, TypeError):
        raise VLMProviderError("vlm_output_invalid") from None


class ExternalVLMProvider(BaseVLM):
    """Delegate perception while keeping prompts, models and transports private."""

    provider = "external-adapter"
    model = "external-adapter"

    def __init__(self, factory_env: str = "SCOUT_PERCEPTION_ADAPTER") -> None:
        self.factory_env = str(factory_env).strip()
        if not self.factory_env:
            raise ValueError("factory_env must be a non-empty string")
        try:
            self._adapter = load_external_adapter(self.factory_env, "describe")
        except ExternalAdapterError as exc:
            raise VLMProviderError("adapter_not_configured") from None

    def describe(self, image_path: Path, request_id: str) -> VLMResult:
        try:
            image = image_path.read_bytes()
        except OSError:
            raise VLMProviderError("image_invalid") from None
        return self.describe_bytes(image, request_id, content_type=_content_type(image_path.suffix))

    def describe_bytes(
        self,
        image_bytes: bytes,
        request_id: str,
        *,
        content_type: str = "image/jpeg",
        mode: str = "targeted",
    ) -> VLMResult:
        if not isinstance(image_bytes, bytes) or not image_bytes:
            raise VLMProviderError("image_invalid")
        request = {
            "contract": "scout.scene-description.v1",
            "request_id": str(request_id),
            "content_type": str(content_type),
            "mode": str(mode),
        }
        started = time.perf_counter()
        try:
            response = self._adapter.describe(image_bytes, request)
            scene = parse_scene_response(response)
        except VLMProviderError:
            raise
        except Exception:
            raise VLMProviderError("adapter_request_failed") from None
        return VLMResult(
            provider=self.provider,
            latency_seconds=time.perf_counter() - started,
            scene_description=scene,
        )


def _content_type(suffix: str) -> str:
    return "image/png" if suffix.lower() == ".png" else "image/jpeg"
