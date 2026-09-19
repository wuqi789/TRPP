from __future__ import annotations

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from PIL import Image

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

import batch_main
import main as standalone_main
from tests.test_schemas import valid_payload


def write_image(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.new("RGB", (2, 2), color=(255, 0, 0)).save(path)


def write_state(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(valid_payload()), encoding="utf-8")


class BatchDiscoveryTests(unittest.TestCase):
    def test_pairs_recursive_inputs_by_relative_stem_in_sorted_order(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            images = root / "images"
            states = root / "states"
            write_image(images / "z.png")
            write_image(images / "nested" / "a.jpg")
            write_state(states / "z.json")
            write_state(states / "nested" / "a.json")

            pairs = batch_main.discover_pairs(images, states)

            self.assertEqual([pair.key for pair in pairs], ["nested/a", "z"])
            self.assertEqual(pairs[0].image_path, (images / "nested/a.jpg").resolve())
            self.assertEqual(pairs[0].state_path, (states / "nested/a.json").resolve())

    def test_rejects_missing_pair_before_processing(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            images = root / "images"
            states = root / "states"
            write_image(images / "one.jpg")
            write_image(images / "two.jpg")
            write_state(states / "one.json")

            with self.assertRaisesRegex(ValueError, "images without state JSON: two"):
                batch_main.discover_pairs(images, states)

    def test_rejects_duplicate_image_stems(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            images = root / "images"
            states = root / "states"
            write_image(images / "same.jpg")
            write_image(images / "same.png")
            write_state(states / "same.json")

            with self.assertRaisesRegex(ValueError, "duplicate image key: same"):
                batch_main.discover_pairs(images, states)

    def test_prevalidation_rejects_invalid_state(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = root / "images/item.jpg"
            state = root / "states/item.json"
            write_image(image)
            state.parent.mkdir(parents=True)
            state.write_text("{}", encoding="utf-8")
            pair = batch_main.BatchPair("item", image, state)

            with self.assertRaisesRegex(ValueError, "item has invalid state JSON"):
                batch_main._prevalidate_pairs([pair])


class BatchExecutionTests(unittest.TestCase):
    def test_run_batch_rejects_unsafe_batch_id(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            pair = batch_main.BatchPair(
                "item",
                root / "images/item.jpg",
                root / "states/item.json",
            )
            with self.assertRaisesRegex(ValueError, "batch_id must contain"):
                batch_main.run_batch(
                    [pair], root / "decisions", batch_id="../outside"
                )

    def test_run_batch_continues_after_failure_and_preserves_order(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            first = batch_main.BatchPair(
                "a",
                root / "images/a.jpg",
                root / "states/a.json",
            )
            second = batch_main.BatchPair(
                "nested/b",
                root / "images/nested/b.png",
                root / "states/nested/b.json",
            )
            completed_record = {"status": "completed", "decision": {"reason": "ok"}}
            failed_record = {
                "status": "failed",
                "failure": {"stage": "perception", "code": "remote_timeout"},
            }
            calls: list[dict[str, str]] = []

            def fake_run_pipeline(**options: str) -> tuple[int, dict[str, object]]:
                calls.append(options)
                if len(calls) == 1:
                    return 0, completed_record
                return 1, failed_record

            with (
                patch.object(batch_main, "MODULE_ROOT", root),
                patch.object(batch_main, "run_pipeline", side_effect=fake_run_pipeline),
                patch.object(
                    batch_main,
                    "_timestamp",
                    return_value="2026-08-09T10:00:01.000000Z",
                ),
            ):
                record = batch_main.run_batch(
                    [first, second],
                    root / "decisions",
                    batch_id="batch-1",
                    started_at="2026-08-09T10:00:00.000000Z",
                )

            self.assertEqual(record["status"], "partial")
            self.assertEqual(record["completed"], 1)
            self.assertEqual(record["failed"], 1)
            self.assertEqual([item["key"] for item in record["items"]], ["a", "nested/b"])
            self.assertEqual(
                record["items"][0]["output_path"],
                "decisions/batch-1/a.json",
            )
            self.assertNotIn("output_path", record["items"][1])
            self.assertEqual(
                calls[1]["output_path_override"],
                "decisions/batch-1/nested/b.json",
            )
            self.assertEqual(batch_main._batch_exit_code(record), 2)

    def test_unexpected_item_failure_is_sanitized(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            pair = batch_main.BatchPair(
                "item",
                root / "images/item.jpg",
                root / "states/item.json",
            )
            with (
                patch.object(batch_main, "MODULE_ROOT", root),
                patch.object(batch_main, "run_pipeline", return_value=(1, None)),
            ):
                record = batch_main.run_batch(
                    [pair], root / "decisions", batch_id="batch-2"
                )

            self.assertEqual(record["status"], "failed")
            self.assertEqual(
                record["items"][0]["failure"],
                {
                    "stage": "batch_runner",
                    "code": "pipeline_unexpected_failure",
                },
            )

    def test_cli_atomically_writes_summary(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            write_image(root / "images/item.jpg")
            write_state(root / "states/item.json")
            expected = {
                "batch_id": "batch-3",
                "status": "completed",
                "total": 1,
                "completed": 1,
                "failed": 0,
                "items": [],
            }
            stdout = io.StringIO()
            with (
                patch.object(batch_main, "MODULE_ROOT", root),
                patch.object(standalone_main, "MODULE_ROOT", root),
                patch.object(batch_main, "run_batch", return_value=expected),
                contextlib.redirect_stdout(stdout),
            ):
                exit_code = batch_main.main(
                    [
                        "--images-dir",
                        "images",
                        "--state-dir",
                        "states",
                        "--output",
                        "results/latest.json",
                        "--decisions-dir",
                        "results/decisions",
                    ]
                )

            self.assertEqual(exit_code, 0)
            output_path = root / "results/latest.json"
            self.assertEqual(json.loads(output_path.read_text(encoding="utf-8")), expected)
            self.assertEqual(json.loads(stdout.getvalue()), expected)


if __name__ == "__main__":
    unittest.main()
