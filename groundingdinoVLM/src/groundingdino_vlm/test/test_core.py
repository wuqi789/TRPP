import numpy as np
import pytest

from groundingdino_vlm.core import (
    Candidate,
    TargetValidationEngine,
    ValidationState,
    normalize_target,
)
from groundingdino_vlm.detectors import DetectorError, ExternalDetector, MockDetector


def candidate() -> Candidate:
    return Candidate("chair", 0.95, (10, 20, 100, 120))


def test_target_normalization_and_validation():
    assert normalize_target("  Supply   Cart ") == ("Supply Cart", "supply cart.")
    assert normalize_target("   ") == ("", "")
    with pytest.raises(ValueError, match="256"):
        normalize_target("x" * 257)
    with pytest.raises(ValueError, match="bbox"):
        Candidate("chair", 0.9, (5, 5, 5, 10))

    engine = TargetValidationEngine()
    snapshot = engine.set_target("  Supply   Cart ")
    assert snapshot.raw_target_label == "  Supply   Cart "
    assert snapshot.target_label == "Supply Cart"


def test_mock_detector_supports_a_configurable_normalized_box():
    detector = MockDetector(
        {
            "confidence": 0.9,
            "bbox_normalized": [0.1, 0.2, 0.5, 0.8],
        }
    )
    candidates = detector.detect(np.zeros((100, 200, 3), dtype=np.uint8), "box.")
    assert candidates == [Candidate("box", 0.9, (20, 20, 100, 80))]

    with pytest.raises(DetectorError, match="normalized xyxy"):
        MockDetector({"bbox_normalized": [0.8, 0.2, 0.5, 0.9]})


def test_external_detector_passes_only_the_public_contract(monkeypatch):
    class Adapter:
        def detect(self, image, request):
            assert image.shape == (10, 20, 3)
            assert request["contract"] == "scout.target-detection.v1"
            assert request["target_label"] == "curtain."
            return {
                "candidates": [
                    {"phrase": "curtain", "confidence": 0.9, "bbox": [1, 2, 10, 9]}
                ]
            }

    import sys
    import types

    module = types.ModuleType("test_detector_adapter")
    module.create = lambda: Adapter()
    sys.modules[module.__name__] = module
    monkeypatch.setenv("SCOUT_TARGET_DETECTOR_ADAPTER", "test_detector_adapter:create")

    detector = ExternalDetector({})
    assert detector.detect(
        np.zeros((10, 20, 3), dtype=np.uint8), "curtain."
    ) == [Candidate("curtain", 0.9, (1, 2, 10, 9))]


def test_project_gate_requests_vlm_then_requires_two_of_three_frames():
    engine = TargetValidationEngine(window_size=3, required_positive=2)
    engine.set_target("chair")

    first = engine.process_detection([candidate()], 12.0)
    assert first.snapshot.state is ValidationState.VLM_PENDING
    assert first.vlm_request is not None

    current, accepted = engine.complete_vlm(
        first.vlm_request.target_revision,
        first.vlm_request.request_id,
        True,
        25.0,
    )
    assert current
    assert accepted.state is ValidationState.DETECTING
    assert accepted.vlm_accepted

    second = engine.process_detection([candidate()], 11.0)
    assert second.snapshot.state is ValidationState.DETECTING
    third = engine.process_detection([candidate()], 10.0)
    assert third.snapshot.state is ValidationState.VERIFIED
    assert third.snapshot.verified


def test_reject_and_provider_failure_are_cached_fail_closed():
    engine = TargetValidationEngine()
    engine.set_target("chair")
    output = engine.process_detection([candidate()], 1.0)
    request = output.vlm_request
    assert request is not None
    current, rejected = engine.complete_vlm(
        request.target_revision, request.request_id, False, 4.0
    )
    assert current
    assert rejected.state is ValidationState.REJECTED
    assert not rejected.verified
    assert engine.process_detection([candidate()], 1.0).vlm_request is None


@pytest.mark.parametrize(
    "error_code",
    ["VLM_TIMEOUT", "VLM_AUTHENTICATION_FAILED", "VLM_RESPONSE_INVALID"],
)
def test_vlm_failure_categories_are_all_cached_fail_closed(error_code):
    engine = TargetValidationEngine()
    engine.set_target("chair")
    request = engine.process_detection([candidate()], 1.0).vlm_request
    assert request is not None
    _, snapshot = engine.complete_vlm(
        request.target_revision,
        request.request_id,
        False,
        3.0,
        error_code=error_code,
    )
    assert snapshot.state is ValidationState.ERROR
    assert not snapshot.verified
    assert engine.process_detection([candidate()], 1.0).vlm_request is None

    engine.set_target("table")
    output = engine.process_detection([candidate()], 1.0)
    request = output.vlm_request
    assert request is not None
    _, failed = engine.complete_vlm(
        request.target_revision,
        request.request_id,
        False,
        5.0,
        error_code="VLM_TIMEOUT",
        message="timed out",
    )
    assert failed.state is ValidationState.ERROR
    assert failed.error_code == "VLM_TIMEOUT"
    assert not failed.verified
    assert engine.process_detection([candidate()], 1.0).vlm_request is None


def test_target_change_invalidates_inflight_response_and_history():
    engine = TargetValidationEngine()
    engine.set_target("chair")
    request = engine.process_detection([candidate()], 1.0).vlm_request
    assert request is not None

    reset = engine.set_target("supply cart")
    assert reset.state is ValidationState.DETECTING
    current, snapshot = engine.complete_vlm(
        request.target_revision, request.request_id, True, 2.0
    )
    assert not current
    assert snapshot.target_label == "supply cart"
    assert not snapshot.vlm_accepted


def test_empty_target_is_idle_and_does_not_start_vlm():
    engine = TargetValidationEngine()
    engine.set_target("")
    output = engine.process_detection([candidate()], 1.0)
    assert output.snapshot.state is ValidationState.IDLE
    assert output.vlm_request is None


def test_no_detection_stays_fail_closed_without_vlm_request():
    engine = TargetValidationEngine()
    engine.set_target("chair")
    output = engine.process_detection([], 1.0)
    assert output.snapshot.state is ValidationState.DETECTING
    assert not output.snapshot.dino_detected
    assert not output.snapshot.verified
    assert output.vlm_request is None


def test_late_detector_result_cannot_pollute_a_new_target():
    engine = TargetValidationEngine()
    first = engine.set_target("chair")
    engine.set_target("table")
    stale = engine.process_detection(
        [candidate()],
        1.0,
        target_revision=first.target_revision,
    )
    assert not stale.current
    assert stale.snapshot.target_label == "table"
    assert not stale.snapshot.dino_detected
    assert stale.vlm_request is None
