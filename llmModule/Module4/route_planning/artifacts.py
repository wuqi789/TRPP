"""Restricted diagnostic artifact persistence for route generation."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re


class ArtifactStore:
    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).expanduser()
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self.root, 0o700)

    def task_dir(self, task_id: str) -> Path:
        safe = re.sub(r"[^A-Za-z0-9_.-]", "_", task_id)[:100] or "task"
        value = self.root / safe
        value.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(value, 0o700)
        return value

    def write_bytes(self, task_id: str, name: str, value: bytes) -> Path:
        path = self.task_dir(task_id) / name
        temporary = path.with_suffix(path.suffix + ".tmp")
        with open(temporary, "wb") as stream:
            os.chmod(temporary, 0o600)
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        return path

    def write_text(self, task_id: str, name: str, value: str) -> Path:
        return self.write_bytes(task_id, name, value.encode("utf-8"))

    def write_json(self, task_id: str, name: str, value) -> Path:
        return self.write_text(
            task_id, name, json.dumps(value, ensure_ascii=False, indent=2) + "\n"
        )
