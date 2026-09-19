"""Strict waypoint response parsing with the public adapter boundary."""

from __future__ import annotations

import json

from semantic_navigation_adapters.vlm import (
    ExternalVLMAdapter,
    MockVLMAdapter,
    VLMConfigurationError,
    VLMError,
    VLMTransportError,
)


def parse_waypoints(raw: str, maximum: int = 24) -> list[tuple[int, int]]:
    if "```" in raw:
        raise VLMError("VLM_RESPONSE_INVALID")
    stripped = raw.strip()
    decoder = json.JSONDecoder()
    try:
        value, end = decoder.raw_decode(stripped)
    except json.JSONDecodeError:
        raise VLMError("VLM_RESPONSE_INVALID") from None
    if stripped[end:].strip():
        raise VLMError("VLM_RESPONSE_INVALID")
    if not isinstance(value, dict) or set(value) != {"waypoints"}:
        raise VLMError("VLM_RESPONSE_INVALID")
    points = value["waypoints"]
    if not isinstance(points, list):
        raise VLMError("VLM_RESPONSE_INVALID")
    if len(points) > maximum:
        raise VLMError("VLM_RESPONSE_INVALID")
    output = []
    for index, point in enumerate(points):
        if not isinstance(point, dict) or set(point) != {"x_px", "y_px"}:
            raise VLMError("VLM_RESPONSE_INVALID")
        x_px, y_px = point["x_px"], point["y_px"]
        if (
            isinstance(x_px, bool)
            or isinstance(y_px, bool)
            or not isinstance(x_px, int)
            or not isinstance(y_px, int)
        ):
            raise VLMError("VLM_RESPONSE_INVALID")
        output.append((x_px, y_px))
    return output


__all__ = [
    "ExternalVLMAdapter",
    "MockVLMAdapter",
    "VLMConfigurationError",
    "VLMError",
    "VLMTransportError",
    "parse_waypoints",
]
