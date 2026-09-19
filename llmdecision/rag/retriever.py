"""Abstract retrieval contract and a deterministic local JSON retriever."""

from __future__ import annotations

import re
from abc import ABC, abstractmethod
from dataclasses import dataclass

from rag.knowledge_base import KnowledgeBase, KnowledgeCase

_STOP_WORDS = {
    "a",
    "an",
    "and",
    "are",
    "in",
    "is",
    "of",
    "on",
    "the",
    "to",
    "was",
    "with",
}


def _tokens(*values: str | None) -> set[str]:
    text = " ".join(value or "" for value in values).lower()
    text = text.replace("_", " ").replace("-", " ")
    return {
        token
        for token in re.findall(r"[^\W_]+", text, flags=re.UNICODE)
        if token not in _STOP_WORDS
    }


@dataclass(frozen=True)
class RetrievalResult:
    case: KnowledgeCase
    score: float
    matched_terms: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        result = self.case.to_dict()
        result["relevance_score"] = round(self.score, 6)
        result["matched_terms"] = list(self.matched_terms)
        return result


class BaseRetriever(ABC):
    """Interface implemented by local and future vector-database retrievers."""

    @abstractmethod
    def retrieve(
        self,
        query_text: str,
        top_k: int = 3,
    ) -> list[RetrievalResult]:
        """Return experience cases relevant to a natural-language scene query."""


class LocalJSONRetriever(BaseRetriever):
    """Explainable token matching over a validated local knowledge base."""

    def __init__(self, knowledge_base: KnowledgeBase) -> None:
        self._knowledge_base = knowledge_base

    def retrieve(
        self,
        query_text: str,
        top_k: int = 3,
    ) -> list[RetrievalResult]:
        if not isinstance(query_text, str) or not query_text.strip():
            raise ValueError("query_text must be a non-empty string")
        if isinstance(top_k, bool) or not isinstance(top_k, int) or top_k <= 0:
            raise ValueError("top_k must be a positive integer")

        query_tokens = _tokens(query_text)
        ranked: list[tuple[int, RetrievalResult]] = []

        for index, case in enumerate(self._knowledge_base.cases):
            class_tokens = _tokens(case.object_class)
            material_tokens = _tokens(case.material)
            keyword_tokens = _tokens(*case.keywords)
            description_tokens = _tokens(case.description)
            all_case_tokens = (
                class_tokens | material_tokens | keyword_tokens | description_tokens
            )
            matched = query_tokens & all_case_tokens

            class_score = (
                len(query_tokens & class_tokens) / len(class_tokens)
                if class_tokens
                else 0.0
            )
            material_score = (
                len(query_tokens & material_tokens) / len(material_tokens)
                if material_tokens
                else 0.0
            )
            keyword_score = (
                len(query_tokens & keyword_tokens) / len(keyword_tokens)
                if keyword_tokens
                else 0.0
            )
            union = query_tokens | description_tokens
            description_score = (
                len(query_tokens & description_tokens) / len(union) if union else 0.0
            )
            score = (
                0.45 * class_score
                + 0.20 * material_score
                + 0.25 * keyword_score
                + 0.10 * description_score
            )

            if score > 0.0:
                ranked.append(
                    (
                        index,
                        RetrievalResult(
                            case=case,
                            score=min(1.0, score),
                            matched_terms=tuple(sorted(matched)),
                        ),
                    )
                )

        ranked.sort(key=lambda item: (-item[1].score, item[0]))
        return [item[1] for item in ranked[:top_k]]
