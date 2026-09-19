"""Run the image-to-decision pipeline for matched image and state files."""

from __future__ import annotations

import argparse
import json
import logging
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

from api.input_schema import DecisionContext, SchemaValidationError
from core.result_store import _atomic_write_json
from main import (
    MODULE_ROOT,
    _inspect_image,
    _load_json,
    _resolve_module_path,
    _timestamp,
    run_pipeline,
)


LOGGER = logging.getLogger(__name__)
DEFAULT_IMAGES_DIR = "examples/images"
DEFAULT_STATE_DIR = "examples/state_machine"
DEFAULT_OUTPUT_PATH = "runtime/batch_output.json"
DEFAULT_DECISIONS_DIR = "runtime/batch_decisions"
IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png"})


@dataclass(frozen=True)
class BatchPair:
    """One image and state document sharing the same relative stem."""

    key: str
    image_path: Path
    state_path: Path


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run llmdecision/main.py for matched image and state JSON files."
    )
    parser.add_argument(
        "--images-dir",
        default=DEFAULT_IMAGES_DIR,
        help="Image directory relative to llmdecision",
    )
    parser.add_argument(
        "--state-dir",
        default=DEFAULT_STATE_DIR,
        help="State JSON directory relative to llmdecision",
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_PATH,
        help="Batch summary JSON path relative to llmdecision",
    )
    parser.add_argument(
        "--decisions-dir",
        default=DEFAULT_DECISIONS_DIR,
        help="Per-item decision directory relative to llmdecision",
    )
    return parser


def _directory(value: str, path: str) -> Path:
    directory = _resolve_module_path(value, path)
    if not directory.is_dir():
        raise ValueError(f"{path} must be an existing directory")
    return directory


def _collect_files(
    directory: Path,
    suffixes: frozenset[str],
    label: str,
) -> dict[str, Path]:
    collected: dict[str, Path] = {}
    for candidate in sorted(
        directory.rglob("*"),
        key=lambda item: item.relative_to(directory).as_posix(),
    ):
        if not candidate.is_file() or candidate.suffix.lower() not in suffixes:
            continue
        resolved = candidate.resolve()
        try:
            relative = resolved.relative_to(directory)
        except ValueError:
            raise ValueError(f"{label} file must remain inside {label} directory") from None
        key = relative.with_suffix("").as_posix()
        if key in collected:
            raise ValueError(f"duplicate {label} key: {key}")
        collected[key] = resolved
    if not collected:
        raise ValueError(f"{label} directory contains no supported files")
    return collected


def _key_list(keys: set[str]) -> str:
    ordered = sorted(keys)
    shown = ordered[:5]
    suffix = f" (+{len(ordered) - len(shown)} more)" if len(ordered) > 5 else ""
    return ", ".join(shown) + suffix


def discover_pairs(images_dir: Path, state_dir: Path) -> list[BatchPair]:
    """Pair recursively discovered inputs by their relative filename stem."""

    images = _collect_files(images_dir, IMAGE_SUFFIXES, "image")
    states = _collect_files(state_dir, frozenset({".json"}), "state")
    missing_states = set(images) - set(states)
    missing_images = set(states) - set(images)
    if missing_states or missing_images:
        details = []
        if missing_states:
            details.append(f"images without state JSON: {_key_list(missing_states)}")
        if missing_images:
            details.append(f"state JSON without image: {_key_list(missing_images)}")
        raise ValueError("; ".join(details))
    return [
        BatchPair(key, images[key], states[key])
        for key in sorted(images)
    ]


def _module_relative(path: Path) -> str:
    return path.relative_to(MODULE_ROOT).as_posix()


def _validate_output_locations(
    images_dir: Path,
    state_dir: Path,
    output_path: Path,
    decisions_dir: Path,
) -> None:
    for path, label in (
        (output_path, "config.batch.output"),
        (decisions_dir, "config.batch.decisions_dir"),
    ):
        if path == images_dir or path.is_relative_to(images_dir):
            raise ValueError(f"{label} must not be inside the image directory")
        if path == state_dir or path.is_relative_to(state_dir):
            raise ValueError(f"{label} must not be inside the state directory")
    if decisions_dir.exists() and not decisions_dir.is_dir():
        raise ValueError("config.batch.decisions_dir must be a directory")
    if output_path.exists() and not output_path.is_file():
        raise ValueError("config.batch.output must be a JSON file path")
    if output_path == decisions_dir:
        raise ValueError("config.batch.output and decisions_dir must differ")


def _prevalidate_pairs(pairs: Sequence[BatchPair]) -> None:
    """Validate the complete batch before the first remote request."""

    for pair in pairs:
        try:
            _inspect_image(str(pair.image_path))
        except Exception:
            raise ValueError("batch item has an invalid image") from None
        try:
            DecisionContext.from_dict(_load_json(pair.state_path))
        except (SchemaValidationError, ValueError):
            raise ValueError(f"batch item {pair.key} has invalid state JSON") from None


def run_batch(
    pairs: Sequence[BatchPair],
    decisions_dir: Path,
    *,
    batch_id: str | None = None,
    started_at: str | None = None,
) -> dict[str, Any]:
    """Run every pair sequentially and return a serializable batch record."""

    if not pairs:
        raise ValueError("batch must contain at least one item")
    selected_batch_id = batch_id or str(uuid.uuid4())
    if not selected_batch_id or any(
        not (character.isalnum() or character == "-")
        for character in selected_batch_id
    ):
        raise ValueError("batch_id must contain only letters, numbers, and hyphens")
    selected_started_at = started_at or _timestamp()
    batch_decisions_dir = decisions_dir / selected_batch_id
    items: list[dict[str, Any]] = []
    completed = 0

    for index, pair in enumerate(pairs, start=1):
        decision_path = batch_decisions_dir / f"{pair.key}.json"
        LOGGER.info("Processing batch item %s/%s (%s)", index, len(pairs), pair.key)
        exit_code, record = run_pipeline(
            image=str(pair.image_path),
            input_path_override=_module_relative(pair.state_path),
            output_path_override=_module_relative(decision_path),
        )
        item: dict[str, Any] = {
            "index": index,
            "key": pair.key,
            "image_path": _module_relative(pair.image_path),
            "state_path": _module_relative(pair.state_path),
        }
        if exit_code == 0 and record is not None:
            completed += 1
            item.update(
                {
                    "status": "completed",
                    "output_path": _module_relative(decision_path),
                    "record": record,
                }
            )
        else:
            item["status"] = "failed"
            if record is not None:
                item["record"] = record
            else:
                item["failure"] = {
                    "stage": "batch_runner",
                    "code": "pipeline_unexpected_failure",
                }
        items.append(item)

    failed = len(pairs) - completed
    status = "completed" if failed == 0 else "partial" if completed else "failed"
    return {
        "batch_id": selected_batch_id,
        "started_at": selected_started_at,
        "finished_at": _timestamp(),
        "status": status,
        "total": len(pairs),
        "completed": completed,
        "failed": failed,
        "items": items,
    }


def _batch_exit_code(record: Mapping[str, Any]) -> int:
    return 0 if record.get("status") == "completed" else 2


def main(argv: Sequence[str] | None = None) -> int:
    args = _argument_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        images_dir = _directory(args.images_dir, "config.batch.images_dir")
        state_dir = _directory(args.state_dir, "config.batch.state_dir")
        output_path = _resolve_module_path(args.output, "config.batch.output")
        decisions_dir = _resolve_module_path(
            args.decisions_dir, "config.batch.decisions_dir"
        )
        _validate_output_locations(
            images_dir,
            state_dir,
            output_path,
            decisions_dir,
        )
        pairs = discover_pairs(images_dir, state_dir)
        _prevalidate_pairs(pairs)
        record = run_batch(pairs, decisions_dir)
        _atomic_write_json(output_path, record)
        LOGGER.info("Batch summary written to %s", _module_relative(output_path))
        print(json.dumps(record, ensure_ascii=False, indent=2))
        return _batch_exit_code(record)
    except Exception:
        LOGGER.error("Batch decision failed")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
