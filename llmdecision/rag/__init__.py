"""Replaceable retrieval interfaces and local JSON implementation."""

from rag.knowledge_base import KnowledgeBase, KnowledgeCase
from rag.retriever import BaseRetriever, LocalJSONRetriever, RetrievalResult

__all__ = [
    "BaseRetriever",
    "KnowledgeBase",
    "KnowledgeCase",
    "LocalJSONRetriever",
    "RetrievalResult",
]
