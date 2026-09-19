"""Public mechanical-probe contracts.

The public repository deliberately does not contain the mechanical probing
policy, motion limits, contact classifier, or curtain decision rules.  Those
are implementation details of a separately distributed provider.  The ROS
node keeps importing the historical function names so an installed private
provider can replace this module without changing message contracts.

The default implementations fail closed: they never infer a physical state
and never generate a non-zero arm command.  The public launch uses the
``scout_public_mock`` action server instead of this provider.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class StableVisionVote:
    """Provider result shape retained for the historical node boundary."""

    classification: int
    confidence: float
    votes: int
    samples: int
    reason: str


def _same_width(*values: Sequence[object]) -> int:
    if not values or not values[0]:
        raise ValueError("arrays must be non-empty")
    width = len(values[0])
    if any(len(value) != width for value in values[1:]):
        raise ValueError("arrays must have equal width")
    return width


def stable_vision_vote(
    samples: Sequence[tuple[int, float, str]],
    *,
    window: int = 5,
    minimum_votes: int = 3,
) -> StableVisionVote | None:
    """Return no decision until an external vision provider is installed."""

    if window <= 0 or minimum_votes <= 0 or minimum_votes > window:
        raise ValueError("vision vote bounds are invalid")
    del samples
    return None


def clamp_probability(value: float) -> float:
    """Validate a provider probability without applying a policy."""

    number = float(value)
    if number != number or number in (float("inf"), float("-inf")):
        raise ValueError("probability must be finite")
    return max(0.0, min(1.0, number))


def movable_probability(displacement: float) -> float:
    """No public mechanical classifier is shipped; return an unknown score."""

    del displacement
    return 0.0


def flexible_curtain_probability(
    mobility_probability: float,
    sweep_progress: float,
    resistance_ratio: float,
    anchored_evidence_count: int,
    *,
    minimum_sweep_progress: float = 0.0,
    hard_resistance_ratio: float = 0.0,
) -> float:
    del mobility_probability, sweep_progress, resistance_ratio
    del anchored_evidence_count, minimum_sweep_progress, hard_resistance_ratio
    return 0.0


def effort_resistance_ratio(
    effort: Sequence[float], baseline: Sequence[float], thresholds: Sequence[float]
) -> float:
    """Fail closed instead of exposing the private resistance model."""

    _same_width(effort, baseline, thresholds)
    return 0.0


def interpolate_pose(start: Sequence[float], target: Sequence[float], fraction: float):
    """Preserve pose-array compatibility without implementing a motion policy."""

    _same_width(start, target)
    del fraction
    return [float(value) for value in start]


def rate_limited_joint_target(
    current: Sequence[float],
    target: Sequence[float],
    previous_velocity: Sequence[float],
    *,
    maximum_speed: float,
    maximum_acceleration: float,
    dt: float,
) -> tuple[list[float], list[float]]:
    """Return a hold command; private providers own motion generation."""

    _same_width(current, target, previous_velocity)
    del target, maximum_speed, maximum_acceleration, dt
    return [float(value) for value in current], [0.0] * len(current)


def tracking_limited_joint_target(
    actual: Sequence[float],
    target: Sequence[float],
    previous_command: Sequence[float] | None,
    previous_velocity: Sequence[float],
    *,
    maximum_speed: float,
    maximum_acceleration: float,
    maximum_lead: float,
    dt: float,
) -> tuple[list[float], list[float]]:
    """Return the measured pose and zero velocity in the public build."""

    _same_width(actual, target, previous_velocity)
    if previous_command is not None:
        _same_width(actual, previous_command)
    del target, maximum_speed, maximum_acceleration, maximum_lead, dt
    return [float(value) for value in actual], [0.0] * len(actual)


def scaled_command_lead(maximum_lead: float, phase_speed: float, reference_speed: float) -> float:
    """No command lead is exposed by the public implementation."""

    del maximum_lead, phase_speed, reference_speed
    return 0.0


def effort_trend_prediction(
    samples: Sequence[tuple[float, Sequence[float]]],
    progress: float,
    *,
    window: int = 12,
) -> list[float] | None:
    """Return no private effort prediction."""

    del samples, progress
    if window < 1:
        raise ValueError("window must be positive")
    return None


def anchored_evidence(
    resistance_ratio: float,
    velocity_norm: float,
    tracking_error: float,
    displacement_consistent: bool,
    *,
    hard_resistance_ratio: float = 0.0,
    stalled_velocity_threshold: float = 0.0,
    tracking_error_threshold: float = 0.0,
) -> bool:
    """Do not expose an anchored-object classifier in the public build."""

    del resistance_ratio, velocity_norm, tracking_error
    del displacement_consistent, hard_resistance_ratio
    del stalled_velocity_threshold, tracking_error_threshold
    return False


def joint_progress(start: Sequence[float], target: Sequence[float], current: Sequence[float]) -> tuple[float, float]:
    """Return zero progress and a contract-only tracking error."""

    _same_width(start, target, current)
    error = sum((float(goal) - float(value)) ** 2 for goal, value in zip(target, current)) ** 0.5
    return 0.0, error


def displacement_is_consistent(
    displacements: Sequence[tuple[float, float]],
    push_direction: tuple[float, float],
    *,
    minimum_frames: int = 1,
    minimum_displacement: float = 0.0,
    minimum_cosine: float = 0.0,
) -> bool:
    del displacements, push_direction, minimum_frames, minimum_displacement, minimum_cosine
    return False


def sustained_displacement_magnitude(
    displacements: Sequence[tuple[float, float]],
    *,
    window: int = 1,
    minimum_frames: int = 1,
    minimum_displacement: float = 0.0,
) -> float:
    del displacements, window, minimum_frames, minimum_displacement
    return 0.0
