"""Validated input structures for obstacle pushability reasoning."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


class SchemaValidationError(ValueError):
    """Raised when an external input does not satisfy the JSON contract."""


_NUMBER_PATTERN = r"(?:\d+(?:\.\d+)?|\.\d+)"
_SIZE_PATTERN = re.compile(
    rf"^\s*({_NUMBER_PATTERN})\s*m\s*[\u00d7xX]\s*({_NUMBER_PATTERN})\s*m\s*$"
)
_VELOCITY_PATTERN = re.compile(
    rf"^\s*({_NUMBER_PATTERN})\s*m\s*/\s*s\s*$"
)
_PROVIDER_TEXT_MARKERS = re.compile(
    r"(?i)(?:system\s+prompt|chain\s*[- ]?of\s*[- ]?thought|few\s*[- ]?shot|"
    r"api[_ -]?key|authorization|bearer|ssh|endpoint|model\s+path|internal\s+rule)"
)

SCENE_POSITIONS = frozenset({"front", "front-left", "front-right", "left", "right"})
DISTANCE_LEVELS = frozenset({"near", "medium", "far"})
MOTION_STATES = frozenset({"static", "unknown"})
RISK_LEVELS = frozenset({"low", "medium", "high"})
MATERIALS = frozenset(
    {
        "cardboard",
        "plastic",
        "metal",
        "glass",
        "wood",
        "fabric",
        "rubber",
        "mixed",
        "unknown",
    }
)
SUPPORT_TYPES = frozenset({"wheels", "flat_base", "legs", "suspended", "unknown"})
ATTACHMENT_STATES = frozenset({"unattached", "attached", "unknown"})
FRAGILITY_LEVELS = frozenset({"fragile", "non_fragile", "unknown"})
SIZE_LEVELS = frozenset({"small", "medium", "large", "unknown"})
PATH_RELEVANCE_LEVELS = frozenset(
    {"blocking", "near_path", "not_blocking", "unknown"}
)
SCENE_TYPES = frozenset({"indoor", "outdoor", "unknown"})
LIGHTING_LEVELS = frozenset({"bright", "adequate", "dim", "unknown"})
PATH_CLEARANCE_LEVELS = frozenset({"clear", "partial", "blocked", "unknown"})
FREE_SPACE_LEVELS = frozenset({"none", "limited", "adequate", "unknown"})
PRESENCE_LEVELS = frozenset({"yes", "no", "unknown"})


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SchemaValidationError(f"{path} must be a JSON object")
    return value


def _required(data: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in data:
        raise SchemaValidationError(f"{path}.{key} is required")
    return data[key]


def _number(value: Any, path: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SchemaValidationError(f"{path} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise SchemaValidationError(f"{path} must be a finite number")
    if minimum is not None and result < minimum:
        raise SchemaValidationError(f"{path} must be >= {minimum}")
    return result


def _positive(value: Any, path: str) -> float:
    result = _number(value, path)
    if result <= 0.0:
        raise SchemaValidationError(f"{path} must be > 0")
    return result


def _probability(value: Any, path: str) -> float:
    result = _number(value, path)
    if not 0.0 <= result <= 1.0:
        raise SchemaValidationError(f"{path} must be in [0, 1]")
    return result


def _string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SchemaValidationError(f"{path} must be a non-empty string")
    return value.strip()


def _provider_text(value: Any, path: str) -> str:
    """Keep provider labels bounded and redact accidental internal text."""

    result = _string(value, path)
    if len(result) > 128 or any(ord(character) < 32 for character in result):
        return "provider_text_redacted"
    if _PROVIDER_TEXT_MARKERS.search(result):
        return "provider_text_redacted"
    return result


def _enum_string(value: Any, path: str, allowed: frozenset[str]) -> str:
    result = _string(value, path)
    if result not in allowed:
        choices = ", ".join(sorted(allowed))
        raise SchemaValidationError(f"{path} must be one of: {choices}")
    return result


def _exact_fields(data: Mapping[str, Any], path: str, expected: set[str]) -> None:
    missing = sorted(expected - set(data))
    if missing:
        raise SchemaValidationError(f"{path}.{missing[0]} is required")
    extra = sorted(set(data) - expected)
    if extra:
        raise SchemaValidationError(f"{path}.{extra[0]} is not allowed")


def _string_array(value: Any, path: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise SchemaValidationError(f"{path} must be a JSON array")
    result = tuple(_string(item, f"{path}[{index}]") for index, item in enumerate(value))
    if len(set(result)) != len(result):
        raise SchemaValidationError(f"{path} must contain unique values")
    return result


def _provider_text_array(value: Any, path: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise SchemaValidationError(f"{path} must be a JSON array")
    result = tuple(
        _provider_text(item, f"{path}[{index}]")
        for index, item in enumerate(value)
    )
    if len(set(result)) != len(result):
        raise SchemaValidationError(f"{path} must contain unique values")
    return result


def _parse_size(value: Any, path: str) -> tuple[float, float]:
    if not isinstance(value, str):
        raise SchemaValidationError(
            f"{path} must be a unit string such as '0.8m\u00d70.5m'"
        )
    match = _SIZE_PATTERN.fullmatch(value)
    if match is None:
        raise SchemaValidationError(
            f"{path} must match '<length>m\u00d7<width>m' using \u00d7, x, or X"
        )
    length, width = (float(part) for part in match.groups())
    if length <= 0.0 or width <= 0.0:
        raise SchemaValidationError(f"{path} dimensions must be > 0")
    return length, width


def _parse_velocity(value: Any, path: str) -> float:
    if not isinstance(value, str):
        raise SchemaValidationError(
            f"{path} must be a unit string such as '0.5m/s'"
        )
    match = _VELOCITY_PATTERN.fullmatch(value)
    if match is None:
        raise SchemaValidationError(f"{path} must match '<speed>m/s'")
    return float(match.group(1))


def _format_number(value: float) -> str:
    return format(value, ".12g")


@dataclass(frozen=True)
class VehicleState:
    mass: float
    size_m: tuple[float, float]
    velocity_mps: float
    maximum_push_force_n: float | None = None

    @classmethod
    def from_dict(cls, value: Any, path: str = "vehicle_state") -> "VehicleState":
        data = _mapping(value, path)
        force = data.get("maximum_push_force")
        return cls(
            mass=_positive(_required(data, "mass", path), f"{path}.mass"),
            size_m=_parse_size(_required(data, "size", path), f"{path}.size"),
            velocity_mps=_parse_velocity(
                _required(data, "velocity", path),
                f"{path}.velocity",
            ),
            maximum_push_force_n=(
                _positive(force, f"{path}.maximum_push_force")
                if force is not None else None
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        length, width = self.size_m
        result = {
            "mass": self.mass,
            "size": f"{_format_number(length)}m\u00d7{_format_number(width)}m",
            "velocity": f"{_format_number(self.velocity_mps)}m/s",
        }
        if self.maximum_push_force_n is not None:
            result["maximum_push_force"] = self.maximum_push_force_n
        return result


@dataclass(frozen=True)
class InteractionCondition:
    score: float
    level: str

    @classmethod
    def from_dict(cls, value: Any, path: str) -> "InteractionCondition":
        data = _mapping(value, path)
        return cls(
            score=_probability(_required(data, "score", path), f"{path}.score"),
            level=_string(_required(data, "level", path), f"{path}.level"),
        )

    def to_dict(self) -> dict[str, Any]:
        return {"score": self.score, "level": self.level}


@dataclass(frozen=True)
class RobotInteractionState:
    overall_condition: InteractionCondition
    joint_load_condition: InteractionCondition
    motion_stability: InteractionCondition
    joint_limit_risk: InteractionCondition
    interaction_readiness: InteractionCondition

    @classmethod
    def from_dict(
        cls,
        value: Any,
        path: str = "robot_interaction_state",
    ) -> "RobotInteractionState":
        data = _mapping(value, path)
        return cls(
            overall_condition=InteractionCondition.from_dict(
                _required(data, "overall_condition", path),
                f"{path}.overall_condition",
            ),
            joint_load_condition=InteractionCondition.from_dict(
                _required(data, "joint_load_condition", path),
                f"{path}.joint_load_condition",
            ),
            motion_stability=InteractionCondition.from_dict(
                _required(data, "motion_stability", path),
                f"{path}.motion_stability",
            ),
            joint_limit_risk=InteractionCondition.from_dict(
                _required(data, "joint_limit_risk", path),
                f"{path}.joint_limit_risk",
            ),
            interaction_readiness=InteractionCondition.from_dict(
                _required(data, "interaction_readiness", path),
                f"{path}.interaction_readiness",
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "overall_condition": self.overall_condition.to_dict(),
            "joint_load_condition": self.joint_load_condition.to_dict(),
            "motion_stability": self.motion_stability.to_dict(),
            "joint_limit_risk": self.joint_limit_risk.to_dict(),
            "interaction_readiness": self.interaction_readiness.to_dict(),
        }


@dataclass(frozen=True)
class SceneEnvironment:
    scene_type: str
    ground_surface: str
    ground_condition: str
    lighting: str
    path_clearance: str
    surrounding_free_space: str
    people_present: str
    hazards: tuple[str, ...]

    @classmethod
    def from_dict(
        cls,
        value: Any,
        path: str = "scene_environment",
    ) -> "SceneEnvironment":
        data = _mapping(value, path)
        expected = {
            "scene_type",
            "ground_surface",
            "ground_condition",
            "lighting",
            "path_clearance",
            "surrounding_free_space",
            "people_present",
            "hazards",
        }
        _exact_fields(data, path, expected)
        return cls(
            scene_type=_enum_string(
                data["scene_type"], f"{path}.scene_type", SCENE_TYPES
            ),
            ground_surface=_provider_text(
                data["ground_surface"], f"{path}.ground_surface"
            ),
            ground_condition=_provider_text(
                data["ground_condition"], f"{path}.ground_condition"
            ),
            lighting=_enum_string(
                data["lighting"], f"{path}.lighting", LIGHTING_LEVELS
            ),
            path_clearance=_enum_string(
                data["path_clearance"],
                f"{path}.path_clearance",
                PATH_CLEARANCE_LEVELS,
            ),
            surrounding_free_space=_enum_string(
                data["surrounding_free_space"],
                f"{path}.surrounding_free_space",
                FREE_SPACE_LEVELS,
            ),
            people_present=_enum_string(
                data["people_present"],
                f"{path}.people_present",
                PRESENCE_LEVELS,
            ),
            hazards=_provider_text_array(data["hazards"], f"{path}.hazards"),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "scene_type": self.scene_type,
            "ground_surface": self.ground_surface,
            "ground_condition": self.ground_condition,
            "lighting": self.lighting,
            "path_clearance": self.path_clearance,
            "surrounding_free_space": self.surrounding_free_space,
            "people_present": self.people_present,
            "hazards": list(self.hazards),
        }


@dataclass(frozen=True)
class SceneObstacle:
    category: str
    position: str
    distance_level: str
    motion: str
    risk: str
    material: str = "unknown"
    support_type: str = "unknown"
    attachment_state: str = "unknown"
    fragility: str = "unknown"
    size_level: str = "unknown"
    path_relevance: str = "unknown"

    @classmethod
    def from_dict(cls, value: Any, path: str = "scene_obstacle") -> "SceneObstacle":
        data = _mapping(value, path)
        legacy_fields = {"category", "position", "distance_level", "motion", "risk"}
        expanded_fields = legacy_fields | {
            "material",
            "support_type",
            "attachment_state",
            "fragility",
            "size_level",
            "path_relevance",
        }
        unknown_fields = set(data) - expanded_fields
        if unknown_fields:
            raise SchemaValidationError(
                f"{path}.{sorted(unknown_fields)[0]} is not allowed"
            )
        if set(data) <= legacy_fields:
            _exact_fields(data, path, legacy_fields)
            expanded = {
                **data,
                **{key: "unknown" for key in expanded_fields - legacy_fields},
            }
        else:
            _exact_fields(data, path, expanded_fields)
            expanded = data
        return cls(
            category=_provider_text(expanded["category"], f"{path}.category"),
            position=_enum_string(
                expanded["position"], f"{path}.position", SCENE_POSITIONS
            ),
            distance_level=_enum_string(
                expanded["distance_level"],
                f"{path}.distance_level",
                DISTANCE_LEVELS,
            ),
            motion=_enum_string(
                expanded["motion"], f"{path}.motion", MOTION_STATES
            ),
            risk=_enum_string(expanded["risk"], f"{path}.risk", RISK_LEVELS),
            material=_enum_string(
                expanded["material"], f"{path}.material", MATERIALS
            ),
            support_type=_enum_string(
                expanded["support_type"], f"{path}.support_type", SUPPORT_TYPES
            ),
            attachment_state=_enum_string(
                expanded["attachment_state"],
                f"{path}.attachment_state",
                ATTACHMENT_STATES,
            ),
            fragility=_enum_string(
                expanded["fragility"], f"{path}.fragility", FRAGILITY_LEVELS
            ),
            size_level=_enum_string(
                expanded["size_level"], f"{path}.size_level", SIZE_LEVELS
            ),
            path_relevance=_enum_string(
                expanded["path_relevance"],
                f"{path}.path_relevance",
                PATH_RELEVANCE_LEVELS,
            ),
        )

    def to_dict(self) -> dict[str, str]:
        return {
            "category": self.category,
            "position": self.position,
            "distance_level": self.distance_level,
            "motion": self.motion,
            "risk": self.risk,
            "material": self.material,
            "support_type": self.support_type,
            "attachment_state": self.attachment_state,
            "fragility": self.fragility,
            "size_level": self.size_level,
            "path_relevance": self.path_relevance,
        }


def select_primary_obstacle(obstacles: Sequence[SceneObstacle]) -> int:
    """Return a stable contract default; policy selection belongs to an adapter.

    The public schema validates obstacle metadata but intentionally does not
    rank risk, distance, position, or path relevance.  A private perception or
    decision adapter may provide ``primary_obstacle_index`` explicitly.
    """

    if not obstacles:
        raise SchemaValidationError("scene_description.obstacles must not be empty")
    return 0


def _scene_text(
    obstacles: Sequence[SceneObstacle],
    primary_index: int,
    environment: SceneEnvironment | None = None,
) -> str:
    descriptions = [
        (
            f"{index + 1}) category={obstacle.category}, position={obstacle.position}, "
            f"distance_level={obstacle.distance_level}, motion={obstacle.motion}, "
            f"risk={obstacle.risk}, material={obstacle.material}, "
            f"support_type={obstacle.support_type}, "
            f"attachment_state={obstacle.attachment_state}, "
            f"fragility={obstacle.fragility}, size_level={obstacle.size_level}, "
            f"path_relevance={obstacle.path_relevance}"
        )
        for index, obstacle in enumerate(obstacles)
    ]
    obstacle_text = (
        "Detected obstacles: "
        + "; ".join(descriptions)
        + f". Primary obstacle: {primary_index + 1}."
    )
    if environment is None:
        return obstacle_text
    hazards = ", ".join(environment.hazards) if environment.hazards else "none"
    return (
        "Environment: "
        f"scene_type={environment.scene_type}, "
        f"ground_surface={environment.ground_surface}, "
        f"ground_condition={environment.ground_condition}, "
        f"lighting={environment.lighting}, "
        f"path_clearance={environment.path_clearance}, "
        f"surrounding_free_space={environment.surrounding_free_space}, "
        f"people_present={environment.people_present}, hazards={hazards}. "
        + obstacle_text
    )


@dataclass(frozen=True)
class SceneDescription:
    text: str
    environment: SceneEnvironment | None = None
    obstacles: tuple[SceneObstacle, ...] = ()
    primary_obstacle_index: int | None = None

    @classmethod
    def from_dict(
        cls,
        value: Any,
        path: str = "scene_description",
    ) -> "SceneDescription":
        data = _mapping(value, path)
        text = _string(_required(data, "text", path), f"{path}.text")
        if "obstacles" not in data:
            if "primary_obstacle_index" in data or "environment" in data:
                raise SchemaValidationError(
                    f"{path}.environment and primary_obstacle_index require {path}.obstacles"
                )
            return cls(text=text)

        raw_obstacles = data["obstacles"]
        if not isinstance(raw_obstacles, list):
            raise SchemaValidationError(f"{path}.obstacles must be a JSON array")
        if not raw_obstacles:
            raise SchemaValidationError(f"{path}.obstacles must not be empty")
        obstacles = tuple(
            SceneObstacle.from_dict(item, f"{path}.obstacles[{index}]")
            for index, item in enumerate(raw_obstacles)
        )
        environment = (
            SceneEnvironment.from_dict(data["environment"], f"{path}.environment")
            if "environment" in data
            else None
        )
        primary_index = _required(data, "primary_obstacle_index", path)
        if (
            isinstance(primary_index, bool)
            or not isinstance(primary_index, int)
            or not 0 <= primary_index < len(obstacles)
        ):
            raise SchemaValidationError(
                f"{path}.primary_obstacle_index must index {path}.obstacles"
            )
        return cls(
            text=text,
            environment=environment,
            obstacles=obstacles,
            primary_obstacle_index=primary_index,
        )

    @classmethod
    def from_obstacles(
        cls,
        obstacles: Sequence[SceneObstacle],
    ) -> "SceneDescription":
        parsed = tuple(obstacles)
        primary_index = select_primary_obstacle(parsed)
        return cls(
            text=_scene_text(parsed, primary_index),
            obstacles=parsed,
            primary_obstacle_index=primary_index,
        )

    @classmethod
    def from_perception(
        cls,
        environment: SceneEnvironment,
        obstacles: Sequence[SceneObstacle],
    ) -> "SceneDescription":
        parsed = tuple(obstacles)
        primary_index = select_primary_obstacle(parsed)
        return cls(
            text=_scene_text(parsed, primary_index, environment),
            environment=environment,
            obstacles=parsed,
            primary_obstacle_index=primary_index,
        )

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {"text": self.text}
        if self.obstacles:
            if self.environment is not None:
                result["environment"] = self.environment.to_dict()
            result["obstacles"] = [obstacle.to_dict() for obstacle in self.obstacles]
            result["primary_obstacle_index"] = self.primary_obstacle_index
        return result


@dataclass(frozen=True)
class DecisionContext:
    vehicle_state: VehicleState
    robot_interaction_state: RobotInteractionState

    @classmethod
    def from_dict(cls, value: Any) -> "DecisionContext":
        data = _mapping(value, "input")
        return cls(
            vehicle_state=VehicleState.from_dict(
                _required(data, "vehicle_state", "input")
            ),
            robot_interaction_state=RobotInteractionState.from_dict(
                _required(data, "robot_interaction_state", "input")
            ),
        )

    def with_scene_description(self, scene: SceneDescription) -> "DecisionInput":
        return DecisionInput(
            vehicle_state=self.vehicle_state,
            robot_interaction_state=self.robot_interaction_state,
            scene_description=scene,
        )


@dataclass(frozen=True)
class DecisionInput:
    vehicle_state: VehicleState
    robot_interaction_state: RobotInteractionState
    scene_description: SceneDescription

    @classmethod
    def from_dict(cls, value: Any) -> "DecisionInput":
        data = _mapping(value, "input")
        return cls(
            vehicle_state=VehicleState.from_dict(
                _required(data, "vehicle_state", "input")
            ),
            robot_interaction_state=RobotInteractionState.from_dict(
                _required(data, "robot_interaction_state", "input")
            ),
            scene_description=SceneDescription.from_dict(
                _required(data, "scene_description", "input")
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "vehicle_state": self.vehicle_state.to_dict(),
            "robot_interaction_state": self.robot_interaction_state.to_dict(),
            "scene_description": self.scene_description.to_dict(),
        }
