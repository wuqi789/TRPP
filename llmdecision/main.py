"""Standalone entry point for the obstacle pushability reasoning module."""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import math
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from api.input_schema import DecisionContext, SchemaValidationError
from core.decision_engine import DecisionEngine
from core.fusion import FusionConfig
from core.prompt_builder import PromptBuilder
from core.result_store import ResultStore
from llm.external_provider import ExternalLLMProvider
from llm.mock_llm import MockLLM
from rag.retriever import BaseRetriever, RetrievalResult
from vlm.base import VLMProviderError
from vlm.external_provider import ExternalVLMProvider

try:
    from PIL import Image
except ModuleNotFoundError:  # Reported as a sanitized image-stage failure at runtime.
    Image = None

MODULE_ROOT = Path(__file__).resolve().parent
CONFIG_PATH = MODULE_ROOT / "config" / "config.yaml"
LOGGER = logging.getLogger(__name__)


class PipelineFailure(RuntimeError):
    """A sanitized pipeline failure suitable for persisted audit records."""

    def __init__(
        self,
        stage: str,
        code: str,
    ) -> None:
        self.stage = stage
        self.code = code
        super().__init__(code)


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{path} must be a mapping")
    return value


def _config_string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{path} must be a non-empty string")
    return value.strip()


def _config_number(
    value: Any,
    path: str,
    *,
    minimum: float,
    maximum: float | None = None,
    strict_minimum: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{path} must be finite")
    if (strict_minimum and result <= minimum) or (
        not strict_minimum and result < minimum
    ):
        operator = ">" if strict_minimum else ">="
        raise ValueError(f"{path} must be {operator} {minimum}")
    if maximum is not None and result > maximum:
        raise ValueError(f"{path} must be <= {maximum}")
    return result


def _config_integer(value: Any, path: str, *, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{path} must be an integer >= {minimum}")
    return value


def _config_boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{path} must be a boolean")
    return value


def _resolve_module_path(value: Any, path: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{path} must be a non-empty relative path")
    relative_path = Path(value)
    if relative_path.is_absolute():
        raise ValueError(f"{path} must be relative to llmdecision")
    resolved = (MODULE_ROOT / relative_path).resolve()
    try:
        resolved.relative_to(MODULE_ROOT)
    except ValueError:
        raise ValueError("path must remain inside llmdecision") from None
    return resolved


def _load_yaml(path: Path) -> Mapping[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as file:
            document = yaml.safe_load(file)
    except (OSError, yaml.YAMLError):
        raise ValueError("cannot load config") from None
    return _mapping(document, "config")


def _load_json(path: Path) -> Mapping[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as file:
            document = json.load(file)
    except (OSError, json.JSONDecodeError):
        raise ValueError("cannot load input") from None
    return _mapping(document, "input")


class _NoRetrieval(BaseRetriever):
    """Public provider requests contain no private retrieval corpus."""

    def retrieve(self, query_text: str, top_k: int = 3) -> list[RetrievalResult]:
        del query_text, top_k
        return []


def _build_engine(config: Mapping[str, Any]) -> DecisionEngine:
    llm_config = _mapping(config.get("llm"), "config.llm")
    provider = llm_config.get("provider")
    if provider == "mock":
        llm = MockLLM()
    elif provider == "external":
        llm = ExternalLLMProvider(
            factory_env=_config_string(
                llm_config.get("factory_env", "SCOUT_DECISION_ADAPTER"),
                "config.llm.factory_env",
            )
        )
    else:
        raise ValueError("config.llm.provider must be 'mock' or 'external'")

    fusion_config = FusionConfig.from_dict(
        _mapping(config.get("fusion"), "config.fusion")
    )
    return DecisionEngine(
        llm=llm,
        retriever=_NoRetrieval(),
        prompt_builder=PromptBuilder(),
        fusion_config=fusion_config,
        top_k=1,
    )


def _build_vlm(config: Mapping[str, Any]) -> ExternalVLMProvider:
    vlm_config = _mapping(config.get("vlm"), "config.vlm")
    if vlm_config.get("provider") != "external":
        raise ValueError("config.vlm.provider must be 'external'")
    return ExternalVLMProvider(
        factory_env=_config_string(
            vlm_config.get("factory_env", "SCOUT_PERCEPTION_ADAPTER"),
            "config.vlm.factory_env",
        )
    )


def _inspect_image(value: str) -> tuple[Path, dict[str, Any]]:
    try:
        path = Path(value).expanduser().resolve(strict=True)
    except (OSError, RuntimeError):
        raise PipelineFailure("image", "image_invalid") from None
    if not path.is_file() or path.suffix.lower() not in {".jpg", ".jpeg", ".png"}:
        raise PipelineFailure("image", "image_invalid")
    if Image is None:
        raise PipelineFailure("image", "image_dependency_missing")
    try:
        with Image.open(path) as image:
            if image.format not in {"JPEG", "PNG"}:
                raise PipelineFailure("image", "image_invalid")
            image.verify()
        digest = hashlib.sha256()
        with path.open("rb") as file:
            for chunk in iter(lambda: file.read(1024 * 1024), b""):
                digest.update(chunk)
        size_bytes = path.stat().st_size
    except PipelineFailure:
        raise
    except Exception:
        raise PipelineFailure("image", "image_invalid") from None
    return path, {
        "filename": path.name,
        "size_bytes": size_bytes,
        "sha256": digest.hexdigest(),
    }


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds").replace(
        "+00:00", "Z"
    )


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a validated scene description and obstacle decision."
    )
    parser.add_argument("--image", required=True, help="Local JPEG or PNG image")
    parser.add_argument(
        "--input",
        help="State JSON path relative to llmdecision (defaults to config.runtime.input_path)",
    )
    parser.add_argument(
        "--output",
        help="Latest JSON path relative to llmdecision (defaults to config.runtime.output_path)",
    )
    return parser


def _failed_record(
    *,
    request_id: str,
    timestamp: str,
    image: Mapping[str, Any],
    failure: PipelineFailure,
) -> dict[str, Any]:
    record = {
        "request_id": request_id,
        "timestamp": timestamp,
        "status": "failed",
        "image": dict(image),
        "failure": {"stage": failure.stage, "code": failure.code},
    }
    return record


def run_pipeline(
    *,
    image: str,
    input_path_override: str | None = None,
    output_path_override: str | None = None,
) -> tuple[int, dict[str, Any] | None]:
    """Run one complete decision request and return its persisted record."""

    request_id = str(uuid.uuid4())
    created_at = _timestamp()
    image_summary: dict[str, Any] = {"filename": Path(image).name}
    store: ResultStore | None = None
    try:
        config = _load_yaml(CONFIG_PATH)
        logging_config = _mapping(config.get("logging", {}), "config.logging")
        level_name = str(logging_config.get("level", "INFO")).upper()
        logging.basicConfig(
            level=getattr(logging, level_name, logging.INFO),
            format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        )

        runtime_config = _mapping(config.get("runtime"), "config.runtime")
        input_path = _resolve_module_path(
            input_path_override
            if input_path_override is not None
            else runtime_config.get("input_path"),
            "config.runtime.input_path",
        )
        output_path = _resolve_module_path(
            output_path_override
            if output_path_override is not None
            else runtime_config.get("output_path"),
            "config.runtime.output_path",
        )
        history_dir = _resolve_module_path(
            runtime_config.get("history_dir", "runtime/history"),
            "config.runtime.history_dir",
        )
        if input_path == output_path:
            raise ValueError("config.runtime input and output paths must differ")
        store = ResultStore(output_path, history_dir)

        image_path, image_summary = _inspect_image(image)
        try:
            context = DecisionContext.from_dict(_load_json(input_path))
        except (SchemaValidationError, ValueError):
            raise PipelineFailure("input", "state_input_invalid") from None

        try:
            perception = _build_vlm(config).describe(image_path, request_id)
        except VLMProviderError as exc:
            raise PipelineFailure("perception", exc.code) from None

        decision_input = context.with_scene_description(
            perception.scene_description
        )
        decision = _build_engine(config).decide(decision_input)
        record = {
            "request_id": request_id,
            "timestamp": created_at,
            "status": "completed",
            "image": image_summary,
            "perception": {
                "provider": perception.provider,
                "latency_seconds": round(perception.latency_seconds, 6),
                "scene_description": perception.scene_description.to_dict(),
            },
            "decision": decision.to_dict(),
        }
        history_path = store.write_completed(record)
        LOGGER.info("Decision written to %s", output_path.relative_to(MODULE_ROOT))
        LOGGER.info("History written to %s", history_path.relative_to(MODULE_ROOT))
        return 0, record
    except PipelineFailure as exc:
        record = _failed_record(
            request_id=request_id,
            timestamp=created_at,
            image=image_summary,
            failure=exc,
        )
        if store is not None:
            try:
                history_path = store.write_failed(record)
                LOGGER.info(
                    "Failure history written to %s",
                    history_path.relative_to(MODULE_ROOT),
                )
            except Exception:
                LOGGER.error("Could not persist failure history")
        LOGGER.error("Decision pipeline failed at %s (%s)", exc.stage, exc.code)
        return 1, record
    except Exception:
        LOGGER.error("Decision pipeline failed")
        return 1, None


def main(argv: Sequence[str] | None = None) -> int:
    args = _argument_parser().parse_args(argv)
    exit_code, record = run_pipeline(
        image=args.image,
        input_path_override=args.input,
        output_path_override=args.output,
    )
    if exit_code == 0 and record is not None:
        print(json.dumps(record, ensure_ascii=False, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
