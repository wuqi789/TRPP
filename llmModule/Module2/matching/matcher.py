"""Deterministic entity matching with an optional caller-owned encoder.

The public package intentionally has no model loader or model identifiers.  A
deployment that needs fuzzy matching may inject an encoder from an external
adapter; the default path remains local exact/alias matching.
"""

from __future__ import annotations

import math
import re
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Sequence


def normalize_label(value: str) -> str:
    return " ".join(re.findall(r"[\w]+", value.casefold().replace("_", " ")))


@dataclass(frozen=True)
class EntityMatch:
    canonical_name: str
    method: str
    similarity: float


class EntityMatcher(ABC):
    @abstractmethod
    def match(self, query: str) -> EntityMatch:
        raise NotImplementedError


class HybridEntityMatcher(EntityMatcher):
    def __init__(
        self,
        ontology: Mapping[str, Sequence[str]],
        *,
        minimum_similarity: float = 0.62,
        minimum_margin: float = 0.08,
        encoder: Callable[[Sequence[str]], Any] | None = None,
    ) -> None:
        self.ontology = {
            str(canonical): tuple(dict.fromkeys((str(canonical), *map(str, aliases))))
            for canonical, aliases in ontology.items()
        }
        self.minimum_similarity = float(minimum_similarity)
        self.minimum_margin = float(minimum_margin)
        self._exact: dict[str, str] = {}
        for canonical, labels in self.ontology.items():
            for label in labels:
                normalized = normalize_label(label)
                owner = self._exact.get(normalized)
                if owner is not None and owner != canonical:
                    raise ValueError(f"ontology alias '{label}' belongs to multiple classes")
                self._exact[normalized] = canonical

        self._encoder = encoder
        self._embeddings = None
        self._labels: list[str] = []
        self._owners: list[str] = []
        for canonical, labels in self.ontology.items():
            for label in labels:
                self._labels.append(normalize_label(label))
                self._owners.append(canonical)
        if self._encoder is not None:
            self._embeddings = self._encoder(self._labels)

    def _load_encoder(self) -> None:
        if self._encoder is not None:
            return
        raise RuntimeError(
            "fuzzy entity matching is unavailable in the public build; "
            "inject an encoder from an external adapter"
        )

    def match(self, query: str) -> EntityMatch:
        normalized = normalize_label(query)
        if not normalized:
            raise ValueError("EMPTY_ENTITY_QUERY")
        exact = self._exact.get(normalized)
        if exact is not None:
            method = "canonical" if normalized == normalize_label(exact) else "alias"
            return EntityMatch(exact, method, 1.0)

        self._load_encoder()
        assert self._encoder is not None
        assert self._embeddings is not None
        query_embedding = self._encoder([normalized])[0]
        scores = [
            self._cosine(query_embedding, candidate)
            for candidate in self._embeddings
        ]
        class_scores: dict[str, float] = {}
        for owner, score in zip(self._owners, scores):
            class_scores[owner] = max(score, class_scores.get(owner, -1.0))
        ranking = sorted(class_scores, key=class_scores.get, reverse=True)
        best = class_scores[ranking[0]]
        second = class_scores[ranking[1]] if len(ranking) > 1 else -1.0
        if best < self.minimum_similarity:
            raise ValueError(f"ENTITY_SIMILARITY_TOO_LOW:{best:.3f}")
        if best - second < self.minimum_margin:
            raise ValueError(f"ENTITY_MATCH_AMBIGUOUS:{best:.3f}:{second:.3f}")
        return EntityMatch(ranking[0], "embedding", best)

    @staticmethod
    def _cosine(left: Any, right: Any) -> float:
        try:
            dot = float(left @ right)
            left_norm = float((left @ left) ** 0.5)
            right_norm = float((right @ right) ** 0.5)
        except Exception:
            dot = sum(float(a) * float(b) for a, b in zip(left, right))
            left_norm = math.sqrt(sum(float(a) ** 2 for a in left))
            right_norm = math.sqrt(sum(float(b) ** 2 for b in right))
        if left_norm == 0.0 or right_norm == 0.0:
            return 0.0
        return dot / (left_norm * right_norm)
