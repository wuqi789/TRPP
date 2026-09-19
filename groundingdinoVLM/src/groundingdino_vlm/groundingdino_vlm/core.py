"""ROS-independent state machine for two-stage target verification."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
import math
import threading
from typing import Sequence


class ValidationState(IntEnum):
    IDLE = 0
    DETECTING = 1
    VLM_PENDING = 2
    VERIFIED = 3
    REJECTED = 4
    ERROR = 5


@dataclass(frozen=True)
class Candidate:
    phrase: str
    confidence: float
    bbox: tuple[int, int, int, int]

    def __post_init__(self) -> None:
        if not isinstance(self.phrase, str) or not self.phrase.strip():
            raise ValueError("candidate phrase must be non-empty")
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("candidate confidence must be in [0, 1]")
        x_min, y_min, x_max, y_max = self.bbox
        if min(self.bbox) < 0 or x_min >= x_max or y_min >= y_max:
            raise ValueError("candidate bbox must be a positive xyxy rectangle")


def overlap_over_smaller(candidate: Candidate, roi) -> float:
    """Return overlap relative to the smaller of the visual and LiDAR boxes."""

    x1, y1, x2, y2 = candidate.bbox
    rx1, ry1 = int(roi.x_offset), int(roi.y_offset)
    rx2, ry2 = rx1 + int(roi.width), ry1 + int(roi.height)
    intersection = max(0, min(x2, rx2) - max(x1, rx1)) * max(
        0, min(y2, ry2) - max(y1, ry1)
    )
    candidate_area = max(1, (x2 - x1) * (y2 - y1))
    roi_area = max(1, (rx2 - rx1) * (ry2 - ry1))
    return float(intersection) / float(min(candidate_area, roi_area))


@dataclass(frozen=True)
class VLMRequest:
    target_revision: int
    request_id: int
    target_label: str
    candidates: tuple[Candidate, ...]


@dataclass(frozen=True)
class ValidationSnapshot:
    target_revision: int
    vlm_request_id: int
    raw_target_label: str
    target_label: str
    state: ValidationState
    dino_detected: bool
    vlm_accepted: bool
    verified: bool
    candidates: tuple[Candidate, ...]
    dino_latency_ms: float
    vlm_latency_ms: float
    error_code: str
    message: str


@dataclass(frozen=True)
class EngineOutput:
    snapshot: ValidationSnapshot
    vlm_request: VLMRequest | None = None
    current: bool = True


def normalize_target(value: str) -> tuple[str, str]:
    """Return normalized display text and a provider-neutral target label."""
    if not isinstance(value, str):
        raise ValueError("target label must be a string")
    display = " ".join(value.strip().split())
    if len(display) > 256:
        raise ValueError("target label must not exceed 256 characters")
    caption = display.casefold().rstrip(". ")
    if caption:
        caption += "."
    return display, caption


class TargetValidationEngine:
    """Thread-safe implementation of the original DCT verification gate.

    The VLM is requested on the first DINO-positive frame. A frame contributes
    a positive temporal vote only after that target's cached VLM result is
    ACCEPT. Provider errors are cached as a rejection until the target changes.
    """

    def __init__(self, window_size: int = 3, required_positive: int = 2) -> None:
        if window_size < 1:
            raise ValueError("window_size must be at least one")
        if not 1 <= required_positive <= window_size:
            raise ValueError("required_positive must be within the window")
        self.window_size = int(window_size)
        self.required_positive = int(required_positive)
        self._lock = threading.RLock()
        self._target_label = ""
        self._raw_target_label = ""
        self._target_caption = ""
        self._target_revision = 0
        self._vlm_request_id = 0
        self._vlm_pending = False
        self._vlm_cached: bool | None = None
        self._history: deque[bool] = deque(maxlen=self.window_size)
        self._candidates: tuple[Candidate, ...] = ()
        self._dino_latency_ms = 0.0
        self._vlm_latency_ms = 0.0
        self._error_code = ""
        self._message = "No target label has been provided"

    @property
    def target_caption(self) -> str:
        with self._lock:
            return self._target_caption

    @property
    def target_revision(self) -> int:
        with self._lock:
            return self._target_revision

    @property
    def target_context(self) -> tuple[int, str]:
        """Return one atomic target revision/caption pair for detector work."""
        with self._lock:
            return self._target_revision, self._target_caption

    def set_target(self, value: str) -> ValidationSnapshot:
        display, caption = normalize_target(value)
        with self._lock:
            if display == self._target_label:
                self._raw_target_label = value
                return self._snapshot_locked()
            self._target_revision += 1
            self._vlm_request_id += 1  # invalidates every in-flight response
            self._target_label = display
            self._raw_target_label = value
            self._target_caption = caption
            self._vlm_pending = False
            self._vlm_cached = None
            self._history.clear()
            self._candidates = ()
            self._dino_latency_ms = 0.0
            self._vlm_latency_ms = 0.0
            self._error_code = ""
            self._message = (
                f"Detecting target '{display}'"
                if display
                else "No target label has been provided"
            )
            return self._snapshot_locked()

    def process_detection(
        self,
        candidates: Sequence[Candidate],
        dino_latency_ms: float,
        *,
        target_revision: int | None = None,
    ) -> EngineOutput:
        if not math.isfinite(dino_latency_ms) or dino_latency_ms < 0.0:
            raise ValueError("dino_latency_ms must be finite and non-negative")
        normalized = tuple(candidates)
        with self._lock:
            if target_revision is not None and target_revision != self._target_revision:
                return EngineOutput(self._snapshot_locked(), current=False)
            self._candidates = normalized
            self._dino_latency_ms = float(dino_latency_ms)
            if self._error_code.startswith("DINO_"):
                self._error_code = ""
            if not self._target_label:
                return EngineOutput(self._snapshot_locked())

            detected = bool(normalized)
            request = None
            if detected and self._vlm_cached is None and not self._vlm_pending:
                self._vlm_request_id += 1
                self._vlm_pending = True
                self._error_code = ""
                self._message = "Target candidate found; verification pending"
                request = VLMRequest(
                    self._target_revision,
                    self._vlm_request_id,
                    self._target_label,
                    normalized,
                )

            positive = bool(detected and self._vlm_cached is True)
            self._history.append(positive)
            if not detected and not self._vlm_pending and self._vlm_cached is None:
                self._message = "No detector candidate matches the target"
            elif self._vlm_cached is True and detected:
                positive_count = sum(self._history)
                self._message = (
                    f"Temporal verification {positive_count}/{self.window_size}; "
                    f"requires {self.required_positive}"
                )
            return EngineOutput(self._snapshot_locked(), request)

    def complete_vlm(
        self,
        target_revision: int,
        request_id: int,
        accepted: bool,
        latency_ms: float,
        *,
        error_code: str = "",
        message: str = "",
    ) -> tuple[bool, ValidationSnapshot]:
        if not math.isfinite(latency_ms) or latency_ms < 0.0:
            raise ValueError("latency_ms must be finite and non-negative")
        with self._lock:
            current = (
                target_revision == self._target_revision
                and request_id == self._vlm_request_id
                and self._vlm_pending
            )
            if not current:
                return False, self._snapshot_locked()
            self._vlm_pending = False
            self._vlm_cached = bool(accepted) if not error_code else False
            self._vlm_latency_ms = float(latency_ms)
            self._error_code = str(error_code)
            if error_code:
                self._message = message or "VLM validation failed closed"
            elif accepted:
                self._message = "VLM accepted the target match; awaiting temporal confirmation"
            else:
                self._message = message or "VLM rejected the target match"
            return True, self._snapshot_locked()

    def detector_error(
        self,
        code: str,
        message: str,
        *,
        target_revision: int | None = None,
    ) -> tuple[bool, ValidationSnapshot]:
        with self._lock:
            if target_revision is not None and target_revision != self._target_revision:
                return False, self._snapshot_locked()
            self._history.append(False)
            self._candidates = ()
            self._error_code = str(code) or "DINO_INFERENCE_FAILED"
            self._message = str(message) or "Target detector failed"
            return True, self._snapshot_locked(force_state=ValidationState.ERROR)

    def snapshot(self) -> ValidationSnapshot:
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(
        self, force_state: ValidationState | None = None
    ) -> ValidationSnapshot:
        if force_state is not None:
            state = force_state
        elif not self._target_label:
            state = ValidationState.IDLE
        elif self._vlm_pending:
            state = ValidationState.VLM_PENDING
        elif self._error_code:
            state = ValidationState.ERROR
        elif self._vlm_cached is False:
            state = ValidationState.REJECTED
        elif (
            self._vlm_cached is True
            and len(self._history) == self.window_size
            and sum(self._history) >= self.required_positive
        ):
            state = ValidationState.VERIFIED
        else:
            state = ValidationState.DETECTING
        return ValidationSnapshot(
            target_revision=self._target_revision,
            vlm_request_id=self._vlm_request_id,
            raw_target_label=self._raw_target_label,
            target_label=self._target_label,
            state=state,
            dino_detected=bool(self._candidates),
            vlm_accepted=self._vlm_cached is True,
            verified=state is ValidationState.VERIFIED,
            candidates=self._candidates,
            dino_latency_ms=self._dino_latency_ms,
            vlm_latency_ms=self._vlm_latency_ms,
            error_code=self._error_code,
            message=self._message,
        )
