"""Loading and validation for the local demonstration knowledge base."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping


class KnowledgeBaseError(ValueError):
    """Raised when a local knowledge-base document is malformed."""


def _text(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise KnowledgeBaseError(f"{path} must be a non-empty string")
    return value.strip()


def _text_list(value: Any, path: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise KnowledgeBaseError(f"{path} must be an array")
    return tuple(_text(item, f"{path}[{index}]") for index, item in enumerate(value))


@dataclass(frozen=True)
class KnowledgeCase:
    case_id: str
    object_class: str
    material: str
    description: str
    push_success: bool
    push_force_range: tuple[float, float]
    risks: tuple[str, ...]
    keywords: tuple[str, ...]

    @classmethod
    def from_dict(cls, value: Any, index: int) -> "KnowledgeCase":
        path = f"cases[{index}]"
        if not isinstance(value, Mapping):
            raise KnowledgeBaseError(f"{path} must be an object")
        required = (
            "id",
            "object_class",
            "material",
            "description",
            "push_success",
            "push_force_range",
            "risks",
            "keywords",
        )
        for key in required:
            if key not in value:
                raise KnowledgeBaseError(f"{path}.{key} is required")

        force_range = value["push_force_range"]
        if not isinstance(force_range, list) or len(force_range) != 2:
            raise KnowledgeBaseError(
                f"{path}.push_force_range must contain two numbers"
            )
        parsed_force: list[float] = []
        for force_index, force in enumerate(force_range):
            if isinstance(force, bool) or not isinstance(force, (int, float)):
                raise KnowledgeBaseError(
                    f"{path}.push_force_range[{force_index}] must be a number"
                )
            numeric = float(force)
            if not math.isfinite(numeric) or numeric < 0.0:
                raise KnowledgeBaseError(
                    f"{path}.push_force_range[{force_index}] must be finite and >= 0"
                )
            parsed_force.append(numeric)
        if parsed_force[0] > parsed_force[1]:
            raise KnowledgeBaseError(
                f"{path}.push_force_range must be ordered from minimum to maximum"
            )
        if not isinstance(value["push_success"], bool):
            raise KnowledgeBaseError(f"{path}.push_success must be a boolean")

        return cls(
            case_id=_text(value["id"], f"{path}.id"),
            object_class=_text(value["object_class"], f"{path}.object_class"),
            material=_text(value["material"], f"{path}.material"),
            description=_text(value["description"], f"{path}.description"),
            push_success=value["push_success"],
            push_force_range=(parsed_force[0], parsed_force[1]),
            risks=_text_list(value["risks"], f"{path}.risks"),
            keywords=_text_list(value["keywords"], f"{path}.keywords"),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.case_id,
            "object_class": self.object_class,
            "material": self.material,
            "description": self.description,
            "push_success": self.push_success,
            "push_force_range": list(self.push_force_range),
            "risks": list(self.risks),
            "keywords": list(self.keywords),
        }


class KnowledgeBase:
    """An immutable collection of validated historical experience cases."""

    def __init__(self, cases: Iterable[KnowledgeCase]) -> None:
        self._cases = tuple(cases)
        case_ids = [case.case_id for case in self._cases]
        if len(case_ids) != len(set(case_ids)):
            raise KnowledgeBaseError("knowledge case ids must be unique")

    @property
    def cases(self) -> tuple[KnowledgeCase, ...]:
        return self._cases

    @classmethod
    def from_file(cls, path: Path) -> "KnowledgeBase":
        try:
            with path.open("r", encoding="utf-8") as file:
                document = json.load(file)
        except (OSError, json.JSONDecodeError):
            raise KnowledgeBaseError("cannot load knowledge base") from None

        if isinstance(document, Mapping):
            raw_cases = document.get("cases")
        else:
            raw_cases = document
        if not isinstance(raw_cases, list):
            raise KnowledgeBaseError("knowledge base root must be an array or cases object")
        return cls(
            KnowledgeCase.from_dict(case, index)
            for index, case in enumerate(raw_cases)
        )
