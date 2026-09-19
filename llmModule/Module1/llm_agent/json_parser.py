"""Parse and validate the JSON produced by the language model."""

from __future__ import annotations

import json
import re
import sys
from dataclasses import dataclass
from typing import Any


class NavigationIntentParseError(ValueError):
    """Raised when an LLM response is not a valid navigation intent."""


@dataclass(frozen=True)
class NavigationIntentData:
    goal_object: str
    reference_object: str = ""
    relation: str = ""
    strategy: str = "shortest"
    constraints: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "goal_object": self.goal_object,
            "reference_object": self.reference_object,
            "relation": self.relation,
            "constraints": list(self.constraints),
            "strategy": self.strategy,
        }


def _extract_json(text: str) -> str:
    cleaned = text.strip()
    fenced = re.fullmatch(r"```(?:json)?\s*(.*?)\s*```", cleaned, re.DOTALL | re.IGNORECASE)
    if fenced:
        cleaned = fenced.group(1).strip()
    start, end = cleaned.find("{"), cleaned.rfind("}")
    if start < 0 or end < start:
        print(f"LLM response without JSON object:\n{text}", file=sys.stderr)
        raise NavigationIntentParseError("LLM response does not contain a JSON object")
    return cleaned[start : end + 1]


def parse_navigation_intent(response: str) -> NavigationIntentData:
    """Return a normalized intent, rejecting malformed or action-oriented output."""
    try:
        data = json.loads(_extract_json(response))
    except json.JSONDecodeError as exc:
        raise NavigationIntentParseError(f"Invalid JSON: {exc.msg}") from exc

    if not isinstance(data, dict):
        raise NavigationIntentParseError("Navigation intent must be a JSON object")

    allowed = {
        "goal_object",
        "reference_object",
        "relation",
        "constraints",
        "strategy",
    }
    forbidden = {"velocity", "cmd_vel", "waypoint", "waypoints", "trajectory", "nav2_goal"}
    present = forbidden.intersection(data)
    if present:
        raise NavigationIntentParseError(f"Robot action fields are forbidden: {sorted(present)}")

    unknown = sorted(set(data).difference(allowed))
    if unknown:
        raise NavigationIntentParseError(f"Unexpected navigation intent fields: {unknown}")

    goal = data.get("goal_object")
    if not isinstance(goal, str) or not goal.strip():
        raise NavigationIntentParseError("goal_object must be a non-empty string")

    def optional_string(name: str, default: str = "") -> str:
        value = data.get(name, default)
        if not isinstance(value, str):
            raise NavigationIntentParseError(f"{name} must be a string")
        return value.strip()

    constraints = data.get("constraints", [])
    if not isinstance(constraints, list) or not all(isinstance(item, str) for item in constraints):
        raise NavigationIntentParseError("constraints must be an array of strings")

    return NavigationIntentData(
        goal_object=goal.strip(),
        reference_object=optional_string("reference_object"),
        relation=optional_string("relation"),
        strategy=optional_string("strategy", "shortest") or "shortest",
        constraints=tuple(item.strip() for item in constraints if item.strip()),
    )
