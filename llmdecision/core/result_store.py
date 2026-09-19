"""Atomic persistence for completed decisions and failed perception attempts."""

from __future__ import annotations

import json
import os
import uuid
from pathlib import Path
from typing import Any, Mapping


class ResultStore:
    """Write every attempt to history and only completed attempts to latest."""

    def __init__(self, latest_path: Path, history_dir: Path) -> None:
        self.latest_path = latest_path
        self.history_dir = history_dir

    def write_completed(self, record: Mapping[str, Any]) -> Path:
        history_path = self._write_history(record)
        _atomic_write_json(self.latest_path, record)
        return history_path

    def write_failed(self, record: Mapping[str, Any]) -> Path:
        return self._write_history(record)

    def _write_history(self, record: Mapping[str, Any]) -> Path:
        request_id = record.get("request_id")
        timestamp = record.get("timestamp")
        if not isinstance(request_id, str) or not request_id:
            raise ValueError("record.request_id must be a non-empty string")
        if not isinstance(timestamp, str) or not timestamp:
            raise ValueError("record.timestamp must be a non-empty string")
        safe_timestamp = "".join(character for character in timestamp if character.isalnum())
        safe_request_id = "".join(
            character for character in request_id if character.isalnum() or character == "-"
        )
        if not safe_timestamp or not safe_request_id:
            raise ValueError("record identifiers cannot form a history filename")
        history_path = self.history_dir / f"{safe_timestamp}_{safe_request_id}.json"
        _atomic_write_json(history_path, record)
        return history_path


def _atomic_write_json(path: Path, document: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x", encoding="utf-8") as file:
            os.chmod(temporary, 0o600)
            json.dump(document, file, ensure_ascii=False, indent=2)
            file.write("\n")
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise
