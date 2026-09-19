import threading

import numpy as np

from semantic_navigation_adapters import VLMTransportError

from llm_semantic_affordance.node import Keyframe, PushabilityMapper


class Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)

    def error(self, _message):
        pass


def mapper_shell():
    value = object.__new__(PushabilityMapper)
    value._condition = threading.Condition(threading.RLock())
    value._generation = 0
    value._frozen = False
    value._stopping = False
    value._response_retries = 1
    value._cloud_checked = False
    value._cloud_healthy = False
    value._fatal_error = ""
    value._last_error = ""
    value._logger = Logger()
    value.get_logger = lambda: value._logger
    return value


def keyframe():
    image = np.zeros((2, 2, 3), dtype=np.uint8)
    plane = np.zeros((2, 2), dtype=np.float32)
    return Keyframe(
        generation=0,
        image_bytes=b"png",
        image=image,
        depth=plane,
        class_ids=plane.astype(np.uint8),
        class_confidence=plane,
        self_mask=plane.astype(bool),
        camera=(1.0, 1.0, 0.0, 0.0),
        transform=np.eye(4),
        stamp_ns=1,
    )


def test_transport_exhaustion_does_not_consume_json_correction_round():
    mapper = mapper_shell()
    calls = []

    def fail(_keyframe, _feedback):
        calls.append(1)
        raise VLMTransportError("provider unavailable after 3 attempts")

    mapper._infer_with_transport_retries = fail
    mapper._process_keyframe(keyframe())

    assert len(calls) == 1
    assert mapper._cloud_checked
    assert not mapper._cloud_healthy
    assert mapper._last_error == "provider unavailable after 3 attempts"


def test_late_response_after_freeze_cannot_change_cloud_state():
    mapper = mapper_shell()

    def freeze_then_return(_keyframe, _feedback):
        mapper._frozen = True
        mapper._generation += 1
        return '{"objects":[]}'

    mapper._infer_with_transport_retries = freeze_then_return
    mapper._process_keyframe(keyframe())

    assert not mapper._cloud_checked
    assert not mapper._cloud_healthy
    assert mapper._last_error == ""
