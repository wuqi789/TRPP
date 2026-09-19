"""Load an installed provider adapter without exposing its implementation here."""

from __future__ import annotations

import importlib
import os
import re
from typing import Any


_SPEC = re.compile(r"^[A-Za-z_][A-Za-z0-9_.]*:[A-Za-z_][A-Za-z0-9_]*$")


class ExternalAdapterError(RuntimeError):
    """Sanitized adapter loading or invocation failure."""


def load_external_adapter(factory_env: str, required_method: str) -> Any:
    """Instantiate ``module:factory`` from an environment variable.

    The private package owns credentials, prompts, transports, model selection and
    retry policy. The public workspace only validates the narrow callable boundary.
    """

    spec = os.getenv(factory_env, "").strip()
    if not spec:
        raise ExternalAdapterError("external adapter is not configured")
    if not _SPEC.fullmatch(spec):
        raise ExternalAdapterError("external adapter spec must use module:factory")
    module_name, factory_name = spec.split(":", 1)
    try:
        module = importlib.import_module(module_name)
        factory = getattr(module, factory_name)
        adapter = factory()
    except Exception:
        raise ExternalAdapterError("external adapter initialization failed") from None
    if not callable(getattr(adapter, required_method, None)):
        raise ExternalAdapterError("external adapter contract is not implemented")
    return adapter
