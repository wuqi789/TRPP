#!/usr/bin/env python3
"""Local dependency preflight for optional external perception adapters."""

from __future__ import annotations

import os
from pathlib import Path


def main() -> int:
    root = Path(os.environ.get("GROUNDINGDINO_VLM_ROOT", Path(__file__).parent)).resolve()
    print(f"[INFO] adapter package root: {root}")
    print("[INFO] provider implementation and model assets are supplied externally")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
