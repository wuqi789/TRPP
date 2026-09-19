from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from latency_test import (
    _batch_exit_code,
    _latency_histogram,
    _load_batch,
    _latency_statistics,
    _percentile,
    _render_terminal_report,
    _select_outcome,
    run_batch_benchmark,
)
from main import _resolve_module_path
from tests.test_schemas import valid_payload


class FakeDecision:
    def __init__(
        self,
        distribution: dict[str, float] | None = None,
        risk_flags: list[str] | None = None,
    ) -> None:
        self._distribution = distribution or {
            "push": 0.7,
            "avoid": 0.2,
            "stop": 0.1,
        }
        self._risk_flags = risk_flags or []

    def to_dict(self) -> dict[str, object]:
        return {
            "decision_distribution": self._distribution,
            "risk_flags": self._risk_flags,
        }


class CountingEngine:
    def __init__(self, decision: FakeDecision | None = None) -> None:
        self.calls = 0
        self.decision = decision or FakeDecision()

    def decide(self, payload: dict[str, object]) -> FakeDecision:
        self.calls += 1
        return self.decision


class SequenceEngine:
    def __init__(self, decisions: list[FakeDecision]) -> None:
        self.decisions = decisions
        self.calls = 0

    def decide(self, payload: dict[str, object]) -> FakeDecision:
        decision = self.decisions[self.calls]
        self.calls += 1
        return decision


class LatencyStatisticsTests(unittest.TestCase):
    def test_percentile_interpolates_and_statistics_are_json_safe(self) -> None:
        self.assertEqual(_percentile([1.0], 95), 1.0)
        self.assertAlmostEqual(_percentile([1.0, 3.0], 50), 2.0)
        stats = _latency_statistics([1.0, 2.0, 3.0])
        self.assertEqual(stats["mean_ms"], 2.0)
        self.assertEqual(stats["median_ms"], 2.0)
        self.assertEqual(stats["total_ms"], 6.0)
        self.assertAlmostEqual(stats["throughput_per_second"], 500.0)
        self.assertIn("p95_ms", stats)

    def test_histogram_includes_maximum_and_equal_samples(self) -> None:
        histogram = _latency_histogram([1.0, 2.0, 3.0, 4.0], bin_count=2)
        self.assertEqual([bucket["count"] for bucket in histogram], [2, 2])
        self.assertEqual(sum(bucket["count"] for bucket in histogram), 4)

        equal_histogram = _latency_histogram([5.0, 5.0, 5.0])
        self.assertEqual(
            equal_histogram,
            [{"lower_ms": 5.0, "upper_ms": 5.0, "count": 3}],
        )

    def test_terminal_report_contains_visual_sections_and_slowest_samples(self) -> None:
        report = {
            "provider": "mock",
            "model": "fixed",
            "input_path": "examples/input_examples.json",
            "output_path": "examples/output_examples.json",
            "engine_initialization_ms": 3.5,
            "warmup_calls": 1,
            "measured_calls": 3,
            "successful_responses": 2,
            "failed_responses": 1,
            "outcome_counts": {"push": 1, "avoid": 1, "stop": 0, "failed": 1},
            "latency": _latency_statistics([10.0, 20.0, 50.0]),
            "samples": [
                {"index": 1, "elapsed_ms": 10.0, "outcome": "push", "risk_flags": []},
                {"index": 2, "elapsed_ms": 50.0, "outcome": "failed", "risk_flags": ["llm_output_invalid"]},
                {"index": 3, "elapsed_ms": 20.0, "outcome": "avoid", "risk_flags": []},
            ],
        }
        rendered = _render_terminal_report(report, bar_width=10)
        self.assertIn("LLMDECISION BATCH LATENCY REPORT", rendered)
        self.assertIn("LATENCY SUMMARY (ms)", rendered)
        self.assertIn("3.500 ms (excluded from samples)", rendered)
        self.assertIn("LATENCY DISTRIBUTION", rendered)
        self.assertIn("OUTCOMES", rendered)
        self.assertIn("SLOWEST SAMPLES", rendered)
        self.assertIn("llm_output_invalid", rendered)
        self.assertLess(rendered.index("     2        50.000"), rendered.index("     3        20.000"))

    def test_batch_separates_warmup_and_measured_calls(self) -> None:
        engine = CountingEngine()
        outcomes, report = run_batch_benchmark(engine, [{}, {}, {}], warmup=2)
        self.assertEqual(engine.calls, 5)
        self.assertEqual(report["warmup_calls"], 2)
        self.assertEqual(report["measured_calls"], 3)
        self.assertEqual(report["successful_responses"], 3)
        self.assertEqual(report["failed_responses"], 0)
        self.assertEqual(outcomes, ["push", "push", "push"])
        self.assertEqual(len(report["samples"]), 3)

    def test_batch_marks_llm_fallback_as_failed(self) -> None:
        engine = CountingEngine(
            FakeDecision(risk_flags=["llm_provider_failed", "unknown_mass"])
        )
        outcomes, report = run_batch_benchmark(engine, [{}], warmup=0)
        self.assertEqual(report["successful_responses"], 0)
        self.assertEqual(report["failed_responses"], 1)
        self.assertEqual(outcomes, ["failed"])
        self.assertEqual(report["samples"][0]["outcome"], "failed")

    def test_partial_failure_continues_and_requires_exit_code_two(self) -> None:
        engine = SequenceEngine(
            [
                FakeDecision({"push": 0.8, "avoid": 0.1, "stop": 0.1}),
                FakeDecision(risk_flags=["llm_provider_failed"]),
                FakeDecision({"push": 0.1, "avoid": 0.7, "stop": 0.2}),
            ]
        )
        outcomes, report = run_batch_benchmark(engine, [{}, {}, {}], warmup=0)
        self.assertEqual(outcomes, ["push", "failed", "avoid"])
        self.assertEqual(engine.calls, 3)
        self.assertEqual(_batch_exit_code(report), 2)

    def test_invalid_llm_output_is_also_failed(self) -> None:
        decision = FakeDecision(risk_flags=["llm_output_invalid"])
        self.assertEqual(_select_outcome(decision.to_dict()), "failed")

    def test_outcome_uses_largest_probability_and_safe_tie_break(self) -> None:
        self.assertEqual(
            _select_outcome(
                FakeDecision({"push": 0.1, "avoid": 0.8, "stop": 0.1}).to_dict()
            ),
            "avoid",
        )
        self.assertEqual(
            _select_outcome(
                FakeDecision({"push": 0.4, "avoid": 0.4, "stop": 0.2}).to_dict()
            ),
            "avoid",
        )
        self.assertEqual(
            _select_outcome(
                FakeDecision({"push": 0.2, "avoid": 0.4, "stop": 0.4}).to_dict()
            ),
            "stop",
        )

    def test_batch_preserves_one_hundred_item_order(self) -> None:
        engine = CountingEngine()
        outcomes, report = run_batch_benchmark(engine, [{} for _ in range(100)], warmup=0)
        self.assertEqual(len(outcomes), 100)
        self.assertEqual(len(report["samples"]), 100)
        self.assertEqual([item["index"] for item in report["samples"]], list(range(1, 101)))

    def test_batch_rejects_invalid_counts(self) -> None:
        with self.assertRaises(ValueError):
            run_batch_benchmark(CountingEngine(), [], warmup=0)
        with self.assertRaises(ValueError):
            run_batch_benchmark(CountingEngine(), [{}], warmup=-1)

    def test_batch_loader_validates_array_and_every_schema(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "batch.json"
            path.write_text(json.dumps([valid_payload()]), encoding="utf-8")
            self.assertEqual(len(_load_batch(path)), 1)

            path.write_text(json.dumps(valid_payload()), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "JSON array"):
                _load_batch(path)

            path.write_text("[]", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "must not be empty"):
                _load_batch(path)

            path.write_text(json.dumps([{}]), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "item 1 is invalid"):
                _load_batch(path)

            path.write_text("not-json", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "cannot load batch input"):
                _load_batch(path)

    def test_batch_paths_cannot_escape_module_directory(self) -> None:
        with self.assertRaises(ValueError):
            _resolve_module_path("../output_examples.json", "latency.output_path")


if __name__ == "__main__":
    unittest.main()
