"""Provider-neutral LLM interface."""

from abc import ABC, abstractmethod


class LLMProviderError(RuntimeError):
    """Raised when an LLM provider cannot produce a response."""


class BaseLLM(ABC):
    """Minimal contract shared by offline and hosted LLM providers."""

    @abstractmethod
    def generate(self, request: str) -> str:
        """Return one JSON-formatted provider response for the request envelope."""
