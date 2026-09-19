from __future__ import annotations

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml
from PIL import Image

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

import main as standalone_main
from api.input_schema import SceneDescription, SceneEnvironment, SceneObstacle
from tests.test_schemas import valid_payload
from vlm.base import VLMProviderError, VLMResult


class StaticVLM:
    def __init__(self, error: VLMProviderError | None = None) -> None:
        self.error = error
        self.calls: list[tuple[Path, str]] = []

    def describe(self, image_path: Path, request_id: str) -> VLMResult:
        self.calls.append((image_path, request_id))
        if self.error is not None:
            raise VLMProviderError(self.error.code)
        scene = SceneDescription.from_perception(
            SceneEnvironment(
                scene_type="indoor",
                ground_surface="tile",
                ground_condition="dry and level",
                lighting="adequate",
                path_clearance="blocked",
                surrounding_free_space="limited",
                people_present="no",
                hazards=(),
            ),
            (
                SceneObstacle(
                    "plastic food container",
                    "front-left",
                    "near",
                    "static",
                    "medium",
                    material="plastic",
                    support_type="flat_base",
                    attachment_state="unattached",
                    fragility="non_fragile",
                    size_level="small",
                    path_relevance="blocking",
                ),
            )
        )
        return VLMResult("external-adapter", 1.23456789, scene)


class StaticDecision:
    def to_dict(self) -> dict[str, object]:
        return {
            "object_assessment": {"class": "food_container", "confidence": 0.8},
            "pushability_probability": 0.4,
            "decision_distribution": {"push": 0.3, "avoid": 0.6, "stop": 0.1},
            "risk_flags": ["unknown_mass"],
            "uncertainty": 0.5,
            "reason": "Structured test decision.",
        }


class RecordingEngine:
    def __init__(self) -> None:
        self.calls: list[object] = []

    def decide(self, payload: object) -> StaticDecision:
        self.calls.append(payload)
        return StaticDecision()


class MainPipelineTests(unittest.TestCase):
    def setup_runtime(self, root: Path) -> tuple[Path, Path, Path]:
        (root / "config").mkdir()
        (root / "examples").mkdir()
        input_path = root / "examples" / "input.json"
        output_path = root / "examples" / "latest.json"
        config_path = root / "config" / "config.yaml"
        payload = valid_payload()
        payload["scene_description"]["text"] = "stale scene that must be ignored"
        input_path.write_text(json.dumps(payload), encoding="utf-8")
        config_path.write_text(
            yaml.safe_dump(
                {
                    "runtime": {
                        "input_path": "examples/input.json",
                        "output_path": "examples/latest.json",
                        "history_dir": "examples/history",
                    },
                    "logging": {"level": "INFO"},
                }
            ),
            encoding="utf-8",
        )
        image_path = root / "camera.png"
        Image.new("RGB", (2, 2), color=(255, 0, 0)).save(image_path)
        return config_path, image_path, output_path

    def run_main(
        self,
        root: Path,
        config_path: Path,
        image_path: Path,
        vlm: StaticVLM,
        engine: RecordingEngine,
    ) -> tuple[int, str]:
        stdout = io.StringIO()
        with (
            patch.object(standalone_main, "MODULE_ROOT", root),
            patch.object(standalone_main, "CONFIG_PATH", config_path),
            patch.object(standalone_main, "_build_vlm", return_value=vlm),
            patch.object(standalone_main, "_build_engine", return_value=engine),
            contextlib.redirect_stdout(stdout),
        ):
            exit_code = standalone_main.main(["--image", str(image_path)])
        return exit_code, stdout.getvalue()

    def test_success_overrides_stale_scene_and_writes_latest_and_history(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path, image_path, output_path = self.setup_runtime(root)
            vlm = StaticVLM()
            engine = RecordingEngine()

            exit_code, stdout = self.run_main(
                root, config_path, image_path, vlm, engine
            )

            self.assertEqual(exit_code, 0)
            self.assertEqual(len(engine.calls), 1)
            decision_input = engine.calls[0]
            self.assertNotIn("stale scene", decision_input.scene_description.text)
            self.assertEqual(
                decision_input.scene_description.obstacles[0].category,
                "plastic food container",
            )
            self.assertEqual(
                decision_input.scene_description.environment.path_clearance,
                "blocked",
            )

            latest = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(latest["status"], "completed")
            self.assertEqual(latest["image"]["filename"], "camera.png")
            self.assertEqual(len(latest["image"]["sha256"]), 64)
            self.assertEqual(latest["perception"]["latency_seconds"], 1.234568)
            self.assertEqual(
                latest["perception"]["scene_description"]["environment"][
                    "ground_surface"
                ],
                "tile",
            )
            obstacle = latest["perception"]["scene_description"]["obstacles"][0]
            self.assertEqual(obstacle["material"], "plastic")
            self.assertEqual(obstacle["support_type"], "flat_base")
            self.assertEqual(obstacle["path_relevance"], "blocking")
            self.assertNotIn("raw_response_file", latest["perception"])
            history = list((root / "examples" / "history").glob("*.json"))
            self.assertEqual(len(history), 1)
            self.assertEqual(
                json.loads(history[0].read_text(encoding="utf-8")), latest
            )
            self.assertEqual(json.loads(stdout), latest)

    def test_perception_failure_writes_history_and_preserves_latest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path, image_path, output_path = self.setup_runtime(root)
            output_path.write_text('{"previous":true}\n', encoding="utf-8")
            vlm = StaticVLM(VLMProviderError("vlm_output_invalid"))
            engine = RecordingEngine()

            exit_code, stdout = self.run_main(
                root, config_path, image_path, vlm, engine
            )

            self.assertEqual(exit_code, 1)
            self.assertEqual(stdout, "")
            self.assertEqual(engine.calls, [])
            self.assertEqual(
                json.loads(output_path.read_text(encoding="utf-8")),
                {"previous": True},
            )
            history = list((root / "examples" / "history").glob("*.json"))
            self.assertEqual(len(history), 1)
            failure = json.loads(history[0].read_text(encoding="utf-8"))
            self.assertEqual(failure["status"], "failed")
            self.assertEqual(
                failure["failure"],
                {"stage": "perception", "code": "vlm_output_invalid"},
            )
            self.assertNotIn("raw_vlm_response_file", failure)

    def test_invalid_state_is_recorded_without_calling_vlm_or_engine(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path, image_path, _ = self.setup_runtime(root)
            (root / "examples" / "input.json").write_text("{}", encoding="utf-8")
            vlm = StaticVLM()
            engine = RecordingEngine()

            exit_code, _ = self.run_main(root, config_path, image_path, vlm, engine)

            self.assertEqual(exit_code, 1)
            self.assertEqual(vlm.calls, [])
            self.assertEqual(engine.calls, [])
            history = list((root / "examples" / "history").glob("*.json"))
            failure = json.loads(history[0].read_text(encoding="utf-8"))
            self.assertEqual(failure["failure"]["code"], "state_input_invalid")

    def test_invalid_image_is_recorded_before_vlm_call(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path, image_path, _ = self.setup_runtime(root)
            image_path.write_text("not an image", encoding="utf-8")
            vlm = StaticVLM()
            engine = RecordingEngine()

            exit_code, _ = self.run_main(root, config_path, image_path, vlm, engine)

            self.assertEqual(exit_code, 1)
            self.assertEqual(vlm.calls, [])
            history = list((root / "examples" / "history").glob("*.json"))
            failure = json.loads(history[0].read_text(encoding="utf-8"))
            self.assertEqual(
                failure["failure"], {"stage": "image", "code": "image_invalid"}
            )


if __name__ == "__main__":
    unittest.main()
