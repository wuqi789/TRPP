"""Public launch preflight with no provider-specific checks or secrets."""

from __future__ import annotations

import os
from pathlib import Path


def default_workspace() -> str:
    configured = os.getenv("SCOUT_WORKSPACE", "").strip()
    if configured:
        return str(Path(configured).expanduser().resolve())
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "groundingdinoVLM").is_dir() and (candidate / "llmModule").is_dir():
            return str(candidate)
    raise RuntimeError("Cannot determine the Scout workspace; set SCOUT_WORKSPACE")


def assert_preflight(workspace: str, isaac_python: str) -> None:
    del workspace
    if not isaac_python or not Path(isaac_python).is_file():
        raise RuntimeError(
            "ISAACSIM_PYTHON_EXE must point to the local Isaac Sim python.sh"
        )
    print("[PASS] Public traversal preflight: local Isaac executable is available")


def main() -> int:
    try:
        assert_preflight(default_workspace(), os.getenv("ISAACSIM_PYTHON_EXE", ""))
    except RuntimeError:
        print("preflight failed")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
