"""Strict VLM parsing, RGB-D localization, and session instance fusion."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
from typing import Sequence

import numpy as np


class PushabilityResponseError(ValueError):
    """A VLM response or detection cannot enter the instance map."""


@dataclass(frozen=True)
class Detection2D:
    label: str
    bbox: tuple[int, int, int, int]
    pushable: bool
    confidence: float
    reason: str


@dataclass(frozen=True)
class LocalizedObject:
    label: str
    association_label: str
    semantic_id: int
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    footprint: tuple[float, float, float, float]
    pushable: bool
    confidence: float
    reason: str
    stamp_ns: int


@dataclass(frozen=True)
class InstanceSnapshot:
    instance_id: str
    label: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    pushable: bool
    confidence: float
    positive_confidence: float
    negative_confidence: float
    observation_count: int
    last_observed_ns: int
    evidence: str


def parse_pushability_response(
    raw: str,
    width: int,
    height: int,
    maximum_objects: int = 32,
) -> list[Detection2D]:
    if "```" in raw:
        raise PushabilityResponseError("Markdown fences are not allowed")
    stripped = raw.strip()
    decoder = json.JSONDecoder()
    try:
        value, end = decoder.raw_decode(stripped)
    except json.JSONDecodeError as exc:
        raise PushabilityResponseError(
            f"response is not valid JSON: {exc.msg}"
        ) from exc
    if stripped[end:].strip():
        raise PushabilityResponseError("response contains trailing content")
    if not isinstance(value, dict) or set(value) != {"objects"}:
        raise PushabilityResponseError("response must contain exactly 'objects'")
    objects = value["objects"]
    if not isinstance(objects, list):
        raise PushabilityResponseError("objects must be an array")
    if len(objects) > maximum_objects:
        raise PushabilityResponseError(
            f"objects exceeds the limit of {maximum_objects}"
        )
    output: list[Detection2D] = []
    schema = {"label", "bbox_px", "pushable", "confidence", "reason"}
    for index, entry in enumerate(objects):
        if not isinstance(entry, dict) or set(entry) != schema:
            raise PushabilityResponseError(f"objects[{index}] has an invalid schema")
        label = entry["label"]
        reason = entry["reason"]
        pushable = entry["pushable"]
        confidence = entry["confidence"]
        bbox = entry["bbox_px"]
        if not isinstance(label, str) or not label.strip() or len(label.strip()) > 64:
            raise PushabilityResponseError(f"objects[{index}].label is invalid")
        if not isinstance(reason, str) or len(reason) > 256:
            raise PushabilityResponseError(f"objects[{index}].reason is invalid")
        if not isinstance(pushable, bool):
            raise PushabilityResponseError(f"objects[{index}].pushable must be boolean")
        if (
            isinstance(confidence, bool)
            or not isinstance(confidence, (int, float))
            or not math.isfinite(float(confidence))
            or not 0.0 <= float(confidence) <= 1.0
        ):
            raise PushabilityResponseError(f"objects[{index}].confidence is invalid")
        if (
            not isinstance(bbox, list)
            or len(bbox) != 4
            or any(isinstance(item, bool) or not isinstance(item, int) for item in bbox)
        ):
            raise PushabilityResponseError(f"objects[{index}].bbox_px is invalid")
        x1, y1, x2, y2 = bbox
        if not (0 <= x1 < x2 < width and 0 <= y1 < y2 < height):
            raise PushabilityResponseError(f"objects[{index}].bbox_px is out of bounds")
        if (x2 - x1) * (y2 - y1) < 16:
            raise PushabilityResponseError(f"objects[{index}].bbox_px is too small")
        output.append(Detection2D(
            label.strip(), (x1, y1, x2, y2), pushable,
            float(confidence), reason.strip(),
        ))
    return suppress_overlaps(output)


def suppress_overlaps(
    detections: Sequence[Detection2D], threshold: float = 0.70
) -> list[Detection2D]:
    kept: list[Detection2D] = []
    for candidate in sorted(detections, key=lambda item: item.confidence, reverse=True):
        if any(
            normalize_label(candidate.label) == normalize_label(existing.label)
            and bbox_iou(candidate.bbox, existing.bbox) >= threshold
            for existing in kept
        ):
            continue
        kept.append(candidate)
    return kept


class KeyframeScheduler:
    def __init__(
        self,
        minimum_translation: float = 0.5,
        minimum_rotation: float = math.radians(30.0),
        minimum_interval: float = 5.0,
    ) -> None:
        self.minimum_translation = float(minimum_translation)
        self.minimum_rotation = float(minimum_rotation)
        self.minimum_interval = float(minimum_interval)
        self._last: tuple[float, float, float, float] | None = None

    def should_enqueue(self, x: float, y: float, yaw: float, now: float) -> bool:
        if self._last is None:
            return True
        last_x, last_y, last_yaw, last_time = self._last
        if now - last_time < self.minimum_interval:
            return False
        translation = math.hypot(x - last_x, y - last_y)
        rotation = abs(math.atan2(
            math.sin(yaw - last_yaw), math.cos(yaw - last_yaw)
        ))
        return (
            translation >= self.minimum_translation
            or rotation >= self.minimum_rotation
        )

    def mark_enqueued(self, x: float, y: float, yaw: float, now: float) -> None:
        self._last = (float(x), float(y), float(yaw), float(now))


def localize_detection(
    detection: Detection2D,
    depth: np.ndarray,
    class_ids: np.ndarray,
    class_confidence: np.ndarray,
    self_mask: np.ndarray,
    camera: tuple[float, float, float, float],
    transform: np.ndarray,
    stamp_ns: int,
    *,
    depth_min: float = 0.30,
    depth_max: float = 6.0,
    class_confidence_min: float = 0.35,
    ignored_class_ids: frozenset[int] = frozenset((0, 3, 5)),
    pixel_stride: int = 2,
    minimum_points: int = 20,
    maximum_size: float = 3.5,
) -> LocalizedObject:
    if depth.shape != class_ids.shape or depth.shape != class_confidence.shape:
        raise PushabilityResponseError("depth and semantic image shapes differ")
    if self_mask.shape != depth.shape:
        raise PushabilityResponseError("self mask shape differs from the image")
    x1, y1, x2, y2 = detection.bbox
    rows, columns = np.mgrid[y1:y2 + 1:pixel_stride, x1:x2 + 1:pixel_stride]
    sampled_depth = depth[rows, columns]
    sampled_ids = class_ids[rows, columns]
    sampled_confidence = class_confidence[rows, columns]
    sampled_self = self_mask[rows, columns]
    valid = (
        np.isfinite(sampled_depth)
        & (sampled_depth >= depth_min)
        & (sampled_depth <= depth_max)
        & (sampled_confidence >= class_confidence_min)
        & ~sampled_self
        & ~np.isin(sampled_ids, tuple(ignored_class_ids))
    )
    if int(np.count_nonzero(valid)) < minimum_points:
        raise PushabilityResponseError(
            f"{detection.label} has insufficient valid semantic depth points"
        )
    labels, counts = np.unique(sampled_ids[valid], return_counts=True)
    semantic_id = int(labels[int(np.argmax(counts))])
    valid &= sampled_ids == semantic_id
    median_depth = float(np.median(sampled_depth[valid]))
    depth_band = max(0.20, median_depth * 0.15)
    valid &= np.abs(sampled_depth - median_depth) <= depth_band
    if int(np.count_nonzero(valid)) < minimum_points:
        raise PushabilityResponseError(
            f"{detection.label} depth cluster is too sparse"
        )

    z = sampled_depth[valid].astype(np.float64)
    u = columns[valid].astype(np.float64)
    v = rows[valid].astype(np.float64)
    fx, fy, cx, cy = camera
    camera_points = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
    homogeneous = np.column_stack((camera_points, np.ones(len(camera_points))))
    world = (homogeneous @ transform.T)[:, :3]
    lower = np.percentile(world, 5.0, axis=0)
    upper = np.percentile(world, 95.0, axis=0)
    size = np.maximum(upper - lower, 0.03)
    if np.any(~np.isfinite(size)) or float(np.max(size)) > maximum_size:
        raise PushabilityResponseError(f"{detection.label} has an invalid 3D size")
    center = (lower + upper) / 2.0
    return LocalizedObject(
        label=detection.label,
        association_label=str(semantic_id),
        semantic_id=semantic_id,
        center=tuple(float(value) for value in center),
        size=tuple(float(value) for value in size),
        footprint=(float(lower[0]), float(lower[1]), float(upper[0]), float(upper[1])),
        pushable=detection.pushable,
        confidence=detection.confidence,
        reason=detection.reason,
        stamp_ns=int(stamp_ns),
    )


class _Record:
    def __init__(self, instance_id: str, value: LocalizedObject) -> None:
        self.instance_id = instance_id
        self.label = value.label
        self.association_label = value.association_label
        self.center = np.asarray(value.center, dtype=np.float64)
        self.size = np.asarray(value.size, dtype=np.float64)
        self.footprint = value.footprint
        self.positive_confidence = value.confidence if value.pushable else 0.0
        self.negative_confidence = value.confidence if not value.pushable else 0.0
        self.positive_reason = value.reason if value.pushable else ""
        self.negative_reason = value.reason if not value.pushable else ""
        self.observation_count = 1
        self.last_observed_ns = value.stamp_ns

    def update(self, value: LocalizedObject) -> None:
        weight = 1.0 / min(self.observation_count + 1, 4)
        self.center = (1.0 - weight) * self.center + weight * np.asarray(value.center)
        self.size = (1.0 - weight) * self.size + weight * np.asarray(value.size)
        half = self.size[:2] / 2.0
        self.footprint = (
            float(self.center[0] - half[0]), float(self.center[1] - half[1]),
            float(self.center[0] + half[0]), float(self.center[1] + half[1]),
        )
        if value.pushable and value.confidence > self.positive_confidence:
            self.positive_confidence = value.confidence
            self.positive_reason = value.reason
        if not value.pushable and value.confidence > self.negative_confidence:
            self.negative_confidence = value.confidence
            self.negative_reason = value.reason
        self.observation_count += 1
        self.last_observed_ns = max(self.last_observed_ns, value.stamp_ns)

    def snapshot(self) -> InstanceSnapshot:
        pushable = self.positive_confidence > self.negative_confidence
        confidence = (
            self.positive_confidence if pushable else self.negative_confidence
        )
        evidence = self.positive_reason if pushable else self.negative_reason
        return InstanceSnapshot(
            self.instance_id, self.label,
            tuple(float(value) for value in self.center),
            tuple(float(value) for value in self.size),
            pushable, confidence, self.positive_confidence,
            self.negative_confidence, self.observation_count,
            self.last_observed_ns, evidence,
        )


class InstanceMap:
    def __init__(self, iou_threshold: float = 0.20, distance_threshold: float = 0.60) -> None:
        self.iou_threshold = float(iou_threshold)
        self.distance_threshold = float(distance_threshold)
        self._records: dict[str, _Record] = {}
        self._counters: dict[str, int] = {}

    def update(self, value: LocalizedObject) -> InstanceSnapshot:
        candidates = []
        for record in self._records.values():
            if record.association_label != value.association_label:
                continue
            iou = bbox_iou(record.footprint, value.footprint)
            distance = math.dist(record.center[:2], value.center[:2])
            if iou >= self.iou_threshold or distance <= self.distance_threshold:
                candidates.append((-iou, distance, record.instance_id, record))
        if candidates:
            record = min(candidates)[3]
            record.update(value)
        else:
            base = normalize_label(value.label) or "object"
            self._counters[base] = self._counters.get(base, 0) + 1
            instance_id = f"{base}_{self._counters[base]:04d}"
            record = _Record(instance_id, value)
            self._records[instance_id] = record
        return record.snapshot()

    def snapshots(self) -> tuple[InstanceSnapshot, ...]:
        return tuple(
            self._records[key].snapshot() for key in sorted(self._records)
        )

    def __len__(self) -> int:
        return len(self._records)


def normalize_label(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", value.casefold()).strip("_")


def bbox_iou(left: Sequence[float], right: Sequence[float]) -> float:
    x1 = max(float(left[0]), float(right[0]))
    y1 = max(float(left[1]), float(right[1]))
    x2 = min(float(left[2]), float(right[2]))
    y2 = min(float(left[3]), float(right[3]))
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, float(left[2]) - float(left[0])) * max(
        0.0, float(left[3]) - float(left[1])
    )
    right_area = max(0.0, float(right[2]) - float(right[0])) * max(
        0.0, float(right[3]) - float(right[1])
    )
    union = left_area + right_area - intersection
    return intersection / union if union > 0.0 else 0.0
