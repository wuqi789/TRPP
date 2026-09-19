"""Public contracts for loading private provider adapters."""

from providers.loader import ExternalAdapterError, load_external_adapter

__all__ = ["ExternalAdapterError", "load_external_adapter"]
