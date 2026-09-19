"""Scene-perception provider contracts."""

from vlm.base import BaseVLM, VLMProviderError, VLMResult
from vlm.external_provider import ExternalVLMProvider, parse_scene_response

__all__ = [
    "BaseVLM",
    "ExternalVLMProvider",
    "VLMProviderError",
    "VLMResult",
    "parse_scene_response",
]
