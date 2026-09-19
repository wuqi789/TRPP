#!/usr/bin/env python3
"""Report that semantic model provisioning is an external deployment concern."""

from __future__ import annotations

def main() -> None:
    raise SystemExit(
        "No model is provisioned by the public package. "
        "Install a deployment-owned semantic adapter and inject it explicitly."
    )


if __name__ == "__main__":
    main()
