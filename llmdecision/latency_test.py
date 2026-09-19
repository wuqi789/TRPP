"""Measure batch decision latency without changing pipeline logic.

The measured interval starts immediately before ``DecisionEngine.decide`` and
ends immediately after its structured result is returned.  It therefore covers
    input validation, local RAG retrieval, request-envelope construction, the provider request,
response parsing, and confidence fusion. Batch outcome labels are written in the
same order as the input records.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import statistics
import time
from pathlib import Path
from typing import Any, Mapping, Protocol

from api.input_schema import DecisionInput
from core.result_store import _atomic_write_json
from main import (
    CONFIG_PATH,
    MODULE_ROOT,
    _build_engine,
    _load_yaml,
    _mapping,
    _resolve_module_path,
)

LOGGER = logging.getLogger(__name__)
FALLBACK_RISKS = frozenset({"llm_provider_failed", "llm_output_invalid"})
OUTCOME_PRIORITY = {"push": 0, "avoid": 1, "stop": 2}
DEFAULT_INPUT_PATH = "examples/input_examples.json"
DEFAULT_OUTPUT_PATH = "runtime/latency_outcomes.json"


class DecisionEngineLike(Protocol):
    """Minimal interface required by the latency benchmark."""

    def decide(self, payload: Mapping[str, Any]) -> Any:
        """Return an object exposing ``to_dict()``."""


def _non_negative_integer(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def _percentile(values: list[float], percentile: float) -> float:
    """Return a linearly interpolated percentile for a non-empty sample."""

    if not values:
        raise ValueError("values must not be empty")
    if not 0.0 <= percentile <= 100.0:
        raise ValueError("percentile must be in [0, 100]")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def _latency_statistics(samples_ms: list[float]) -> dict[str, float]:
    if not samples_ms:
        raise ValueError("samples_ms must not be empty")
    total_ms = sum(samples_ms)
    return {
        "min_ms": round(min(samples_ms), 3),
        "mean_ms": round(statistics.fmean(samples_ms), 3),
        "median_ms": round(statistics.median(samples_ms), 3),
        "p90_ms": round(_percentile(samples_ms, 90.0), 3),
        "p95_ms": round(_percentile(samples_ms, 95.0), 3),
        "p99_ms": round(_percentile(samples_ms, 99.0), 3),
        "max_ms": round(max(samples_ms), 3),
        "population_stdev_ms": round(statistics.pstdev(samples_ms), 3),
        "total_ms": round(total_ms, 3),
        "throughput_per_second": round(
            len(samples_ms) * 1000.0 / total_ms if total_ms > 0.0 else 0.0,
            3,
        ),
    }


def _latency_histogram(
    samples_ms: list[float],
    bin_count: int | None = None,
) -> list[dict[str, float | int]]:
    """Build equal-width latency buckets for terminal visualization."""

    if not samples_ms:
        raise ValueError("samples_ms must not be empty")
    if any(not math.isfinite(value) or value < 0.0 for value in samples_ms):
        raise ValueError("samples_ms must contain finite non-negative values")
    if bin_count is None:
        bin_count = min(10, max(1, math.ceil(math.log2(len(samples_ms)) + 1)))
    if (
        isinstance(bin_count, bool)
        or not isinstance(bin_count, int)
        or bin_count <= 0
    ):
        raise ValueError("bin_count must be a positive integer")

    minimum = min(samples_ms)
    maximum = max(samples_ms)
    if math.isclose(minimum, maximum):
        return [{"lower_ms": minimum, "upper_ms": maximum, "count": len(samples_ms)}]

    width = (maximum - minimum) / bin_count
    counts = [0] * bin_count
    for value in samples_ms:
        bucket_index = min(int((value - minimum) / width), bin_count - 1)
        counts[bucket_index] += 1

    return [
        {
            "lower_ms": minimum + index * width,
            "upper_ms": minimum + (index + 1) * width,
            "count": count,
        }
        for index, count in enumerate(counts)
    ]


def _scaled_bar(value: float, maximum: float, width: int) -> str:
    if value <= 0.0 or maximum <= 0.0:
        return ""
    length = max(1, round(value / maximum * width))
    return "#" * min(width, length)


def _render_terminal_report(
    report: Mapping[str, Any],
    bar_width: int = 36,
) -> str:
    """Render a compact ASCII dashboard from a completed benchmark report."""

    if (
        isinstance(bar_width, bool)
        or not isinstance(bar_width, int)
        or bar_width <= 0
    ):
        raise ValueError("bar_width must be a positive integer")
    samples = report.get("samples")
    latency = report.get("latency")
    outcome_counts = report.get("outcome_counts")
    if not isinstance(samples, list) or not samples:
        raise ValueError("report.samples must be a non-empty array")
    if not isinstance(latency, Mapping):
        raise ValueError("report.latency must be an object")
    if not isinstance(outcome_counts, Mapping):
        raise ValueError("report.outcome_counts must be an object")

    elapsed_values = [float(sample["elapsed_ms"]) for sample in samples]
    histogram = _latency_histogram(elapsed_values)
    max_bucket_count = max(int(bucket["count"]) for bucket in histogram)
    measured_calls = int(report.get("measured_calls", len(samples)))
    successful = int(report.get("successful_responses", 0))
    failed = int(report.get("failed_responses", 0))

    dashboard_width = max(72, bar_width + 45)
    lines = [
        "=" * dashboard_width,
        "LLMDECISION BATCH LATENCY REPORT",
        "=" * dashboard_width,
        f"Provider : {report.get('provider', 'unknown')}",
        f"Model    : {report.get('model', 'unknown')}",
        (
            f"Samples  : {measured_calls} measured, {successful} successful, "
            f"{failed} failed, {int(report.get('warmup_calls', 0))} warmup"
        ),
        f"Input    : {report.get('input_path', 'unknown')}",
        f"Output   : {report.get('output_path', 'unknown')}",
        (
            "Init     : "
            f"{float(report.get('engine_initialization_ms', 0.0)):.3f} ms "
            "(excluded from samples)"
        ),
        "",
        "LATENCY SUMMARY (ms)",
    ]

    summary_rows = (
        ("min", "min_ms"),
        ("mean", "mean_ms"),
        ("median", "median_ms"),
        ("p90", "p90_ms"),
        ("p95", "p95_ms"),
        ("p99", "p99_ms"),
        ("max", "max_ms"),
        ("stdev", "population_stdev_ms"),
    )
    maximum_latency = max(float(latency.get(key, 0.0)) for _, key in summary_rows)
    for label, key in summary_rows:
        value = float(latency.get(key, 0.0))
        bar = _scaled_bar(value, maximum_latency, bar_width)
        lines.append(f"  {label:>6} {value:>11.3f} |{bar}")
    lines.extend(
        [
            f"  {'total':>6} {float(latency.get('total_ms', 0.0)):>11.3f}",
            (
                f"  {'rate':>6} "
                f"{float(latency.get('throughput_per_second', 0.0)):>11.3f} samples/s"
            ),
            "",
            "LATENCY DISTRIBUTION",
        ]
    )

    for index, bucket in enumerate(histogram):
        lower = float(bucket["lower_ms"])
        upper = float(bucket["upper_ms"])
        count = int(bucket["count"])
        closing = "]" if index == len(histogram) - 1 else ")"
        interval = f"[{lower:.3f}, {upper:.3f}{closing}"
        percentage = count * 100.0 / measured_calls if measured_calls else 0.0
        bar = _scaled_bar(count, max_bucket_count, bar_width)
        lines.append(
            f"  {interval:>25} |{bar:<{bar_width}} {count:>4} "
            f"({percentage:>5.1f}%)"
        )

    lines.extend(["", "OUTCOMES"])
    max_outcome_count = max(
        (int(value) for value in outcome_counts.values()),
        default=0,
    )
    for outcome in ("push", "avoid", "stop", "failed"):
        count = int(outcome_counts.get(outcome, 0))
        percentage = count * 100.0 / measured_calls if measured_calls else 0.0
        bar = _scaled_bar(count, max_outcome_count, bar_width)
        lines.append(
            f"  {outcome:>6} |{bar:<{bar_width}} {count:>4} "
            f"({percentage:>5.1f}%)"
        )

    lines.extend(["", "SLOWEST SAMPLES"])
    slowest = sorted(
        samples,
        key=lambda sample: float(sample["elapsed_ms"]),
        reverse=True,
    )[:5]
    lines.append("  index    latency_ms  outcome  risk_flags")
    for sample in slowest:
        flags = ",".join(str(flag) for flag in sample.get("risk_flags", [])) or "-"
        lines.append(
            f"  {int(sample['index']):>5} {float(sample['elapsed_ms']):>13.3f}  "
            f"{str(sample['outcome']):<7}  {flags}"
        )
    lines.append("=" * dashboard_width)
    return "\n".join(lines)


def _load_batch(path: Path) -> list[Mapping[str, Any]]:
    """Load and prevalidate every batch item before any LLM request is made."""

    try:
        with path.open("r", encoding="utf-8") as file:
            document = json.load(file)
    except (OSError, json.JSONDecodeError):
        raise ValueError("cannot load batch input") from None
    if not isinstance(document, list):
        raise ValueError("batch input must be a JSON array")
    if not document:
        raise ValueError("batch input must not be empty")

    payloads: list[Mapping[str, Any]] = []
    for index, payload in enumerate(document, start=1):
        if not isinstance(payload, Mapping):
            raise ValueError(f"batch input item {index} must be a JSON object")
        try:
            DecisionInput.from_dict(payload)
        except Exception:
            raise ValueError(f"batch input item {index} is invalid") from None
        payloads.append(payload)
    return payloads


def _select_outcome(serialized: Mapping[str, Any]) -> str:
    """Map one structured decision to push/avoid/stop/failed."""

    risk_flags = serialized.get("risk_flags", [])
    if isinstance(risk_flags, (list, tuple)) and FALLBACK_RISKS.intersection(
        flag for flag in risk_flags if isinstance(flag, str)
    ):
        return "failed"

    distribution = serialized.get("decision_distribution")
    if not isinstance(distribution, Mapping):
        raise ValueError("decision_distribution must be an object")
    probabilities: dict[str, float] = {}
    for action in OUTCOME_PRIORITY:
        value = distribution.get(action)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"decision_distribution.{action} must be a number")
        probability = float(value)
        if not math.isfinite(probability) or not 0.0 <= probability <= 1.0:
            raise ValueError(
                f"decision_distribution.{action} must be a finite number in [0, 1]"
            )
        probabilities[action] = probability
    return max(
        probabilities,
        key=lambda action: (probabilities[action], OUTCOME_PRIORITY[action]),
    )


def run_batch_benchmark(
    engine: DecisionEngineLike,
    payloads: list[Mapping[str, Any]],
    *,
    warmup: int,
) -> tuple[list[str], dict[str, Any]]:
    """Process one batch sequentially and return labels plus a latency report."""

    if not payloads:
        raise ValueError("payloads must not be empty")
    if warmup < 0:
        raise ValueError("warmup must be zero or greater")

    for _ in range(warmup):
        engine.decide(payloads[0])

    samples: list[dict[str, Any]] = []
    elapsed_values: list[float] = []
    outcomes: list[str] = []
    outcome_counts = {"push": 0, "avoid": 0, "stop": 0, "failed": 0}

    for index, payload in enumerate(payloads, start=1):
        started_ns = time.perf_counter_ns()
        decision = engine.decide(payload)
        elapsed_ms = (time.perf_counter_ns() - started_ns) / 1_000_000.0
        serialized = decision.to_dict()
        outcome = _select_outcome(serialized)
        risk_flags = list(serialized.get("risk_flags", []))
        outcomes.append(outcome)
        outcome_counts[outcome] += 1
        elapsed_values.append(elapsed_ms)
        samples.append(
            {
                "index": index,
                "elapsed_ms": round(elapsed_ms, 3),
                "outcome": outcome,
                "risk_flags": risk_flags,
            }
        )
        LOGGER.debug(
            "Completed batch item %s/%s: outcome=%s elapsed_ms=%.3f",
            index,
            len(payloads),
            outcome,
            elapsed_ms,
        )

    report = {
        "measurement_scope": (
            "end_to_end_decision: input validation + local RAG + request envelope + "
            "LLM + parsing + fusion"
        ),
        "clock": "time.perf_counter_ns (monotonic)",
        "warmup_calls": warmup,
        "measured_calls": len(payloads),
        "successful_responses": len(payloads) - outcome_counts["failed"],
        "failed_responses": outcome_counts["failed"],
        "outcome_counts": outcome_counts,
        "latency": _latency_statistics(elapsed_values),
        "samples": samples,
    }
    return outcomes, report


def _batch_exit_code(report: Mapping[str, Any]) -> int:
    """Return 2 for a completed batch containing one or more failed outcomes."""

    return 2 if report.get("failed_responses", 0) else 0


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run batch llmdecision inference and measure feedback latency.",
    )
    parser.add_argument(
        "--warmup",
        type=_non_negative_integer,
        default=0,
        help="unmeasured warmup calls (default: 0; warmups also call the API)",
    )
    parser.add_argument(
        "--input",
        default=DEFAULT_INPUT_PATH,
        help=(
            "batch input JSON path relative to llmdecision "
            f"(default: {DEFAULT_INPUT_PATH})"
        ),
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_PATH,
        help=(
            "label output JSON path relative to llmdecision "
            f"(default: {DEFAULT_OUTPUT_PATH})"
        ),
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="print the complete machine-readable JSON report instead of the dashboard",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _argument_parser().parse_args(argv)
    try:
        config = _load_yaml(CONFIG_PATH)
        logging_config = _mapping(config.get("logging", {}), "config.logging")
        level_name = str(logging_config.get("level", "INFO")).upper()
        logging.basicConfig(
            level=getattr(logging, level_name, logging.INFO),
            format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        )

        input_path = _resolve_module_path(args.input, "latency.input_path")
        output_path = _resolve_module_path(args.output, "latency.output_path")
        if input_path == output_path:
            raise ValueError("latency input and output paths must be different")
        payloads = _load_batch(input_path)

        initialization_started_ns = time.perf_counter_ns()
        engine = _build_engine(config)
        initialization_ms = (
            time.perf_counter_ns() - initialization_started_ns
        ) / 1_000_000.0

        outcomes, report = run_batch_benchmark(
            engine,
            payloads,
            warmup=args.warmup,
        )
        _atomic_write_json(output_path, outcomes)
        llm_config = _mapping(config.get("llm"), "config.llm")
        report = {
            "provider": llm_config.get("provider"),
            "model": llm_config.get("model"),
            "input_path": str(input_path.relative_to(MODULE_ROOT)),
            "output_path": str(output_path.relative_to(MODULE_ROOT)),
            "engine_initialization_ms": round(initialization_ms, 3),
            **report,
        }
        if args.json:
            print(json.dumps(report, ensure_ascii=False, indent=2))
        else:
            print(_render_terminal_report(report))
        LOGGER.info(
            "Batch outcomes written to %s",
            output_path.relative_to(MODULE_ROOT),
        )

        if report["failed_responses"]:
            LOGGER.warning(
                "%s/%s decisions failed to produce usable LLM output",
                report["failed_responses"],
                report["measured_calls"],
            )
        return _batch_exit_code(report)
    except Exception:
        LOGGER.error("Latency benchmark failed")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
