from __future__ import annotations

import sys
import unittest
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from api.input_schema import (
    DecisionContext,
    DecisionInput,
    SceneDescription,
    SceneEnvironment,
    SceneObstacle,
    SchemaValidationError,
    select_primary_obstacle,
)
from api.output_schema import (
    DECISION_JSON_SCHEMA,
    DecisionDistribution,
    DecisionOutput,
    ObjectAssessment,
)


def condition(score: float, level: str) -> dict[str, object]:
    return {"score": score, "level": level}


def valid_payload() -> dict[str, object]:
    return {
        "vehicle_state": {
            "mass": 50,
            "size": "0.8m\u00d70.5m",
            "velocity": "0.5m/s",
        },
        "robot_interaction_state": {
            "overall_condition": condition(0.92, "good"),
            "joint_load_condition": condition(0.85, "low"),
            "motion_stability": condition(0.88, "stable"),
            "joint_limit_risk": condition(0.05, "low"),
            "interaction_readiness": condition(0.85, "high"),
        },
        "scene_description": {
            "text": "A cardboard box blocks the planned path."
        },
    }


def valid_environment() -> SceneEnvironment:
    return SceneEnvironment(
        scene_type="indoor",
        ground_surface="tile",
        ground_condition="dry and level",
        lighting="adequate",
        path_clearance="blocked",
        surrounding_free_space="limited",
        people_present="no",
        hazards=("narrow corridor",),
    )


class InputSchemaTests(unittest.TestCase):
    def test_valid_input_round_trip_uses_new_contract(self) -> None:
        parsed = DecisionInput.from_dict(valid_payload())
        self.assertEqual(parsed.vehicle_state.size_m, (0.8, 0.5))
        self.assertEqual(parsed.vehicle_state.velocity_mps, 0.5)
        self.assertEqual(
            parsed.robot_interaction_state.motion_stability.level,
            "stable",
        )
        self.assertEqual(parsed.to_dict(), valid_payload())

    def test_size_accepts_multiplication_variants_and_spaces(self) -> None:
        for size in ("0.8m\u00d70.5m", "0.8m x 0.5m", " 0.8 m X 0.5 m "):
            with self.subTest(size=size):
                payload = valid_payload()
                payload["vehicle_state"]["size"] = size
                parsed = DecisionInput.from_dict(payload)
                self.assertEqual(parsed.vehicle_state.size_m, (0.8, 0.5))

    def test_invalid_size_unit_is_rejected(self) -> None:
        payload = valid_payload()
        payload["vehicle_state"]["size"] = "80cm by 50cm"
        with self.assertRaisesRegex(SchemaValidationError, "vehicle_state.size"):
            DecisionInput.from_dict(payload)

    def test_invalid_velocity_unit_is_rejected(self) -> None:
        payload = valid_payload()
        payload["vehicle_state"]["velocity"] = "0.5km/h"
        with self.assertRaisesRegex(SchemaValidationError, "vehicle_state.velocity"):
            DecisionInput.from_dict(payload)

    def test_non_positive_mass_is_rejected(self) -> None:
        payload = valid_payload()
        payload["vehicle_state"]["mass"] = 0
        with self.assertRaisesRegex(SchemaValidationError, "vehicle_state.mass"):
            DecisionInput.from_dict(payload)

    def test_optional_maximum_push_force_round_trips_and_is_positive(self) -> None:
        payload = valid_payload()
        payload["vehicle_state"]["maximum_push_force"] = 150
        parsed = DecisionInput.from_dict(payload)
        self.assertEqual(parsed.vehicle_state.maximum_push_force_n, 150.0)
        self.assertEqual(parsed.to_dict(), payload)

        payload["vehicle_state"]["maximum_push_force"] = 0
        with self.assertRaisesRegex(
            SchemaValidationError, "vehicle_state.maximum_push_force"
        ):
            DecisionInput.from_dict(payload)

    def test_out_of_range_interaction_score_is_rejected(self) -> None:
        payload = valid_payload()
        payload["robot_interaction_state"]["motion_stability"]["score"] = 1.2
        with self.assertRaisesRegex(SchemaValidationError, r"\[0, 1\]"):
            DecisionInput.from_dict(payload)

    def test_missing_interaction_condition_has_field_path(self) -> None:
        payload = valid_payload()
        del payload["robot_interaction_state"]["interaction_readiness"]
        with self.assertRaisesRegex(
            SchemaValidationError,
            r"robot_interaction_state\.interaction_readiness",
        ):
            DecisionInput.from_dict(payload)

    def test_empty_scene_text_is_rejected(self) -> None:
        payload = valid_payload()
        payload["scene_description"]["text"] = "  "
        with self.assertRaisesRegex(SchemaValidationError, "scene_description.text"):
            DecisionInput.from_dict(payload)

    def test_legacy_input_is_rejected(self) -> None:
        legacy_payload = {"robot_state": {}, "object_state": {}}
        with self.assertRaisesRegex(SchemaValidationError, r"input\.vehicle_state"):
            DecisionInput.from_dict(legacy_payload)

    def test_runtime_context_ignores_legacy_scene_description(self) -> None:
        context = DecisionContext.from_dict(valid_payload())
        self.assertEqual(context.vehicle_state.mass, 50.0)
        self.assertEqual(context.robot_interaction_state.motion_stability.level, "stable")

    def test_structured_scene_round_trip(self) -> None:
        scene = SceneDescription.from_perception(
            valid_environment(),
            (
                SceneObstacle("cart", "front-right", "far", "static", "low"),
                SceneObstacle("person", "left", "near", "unknown", "high"),
            )
        )
        # Public schema keeps a stable first-obstacle default; policy ranking
        # belongs to the external decision adapter.
        self.assertEqual(scene.primary_obstacle_index, 0)
        self.assertEqual(scene.environment.ground_surface, "tile")
        self.assertIn("path_clearance=blocked", scene.text)
        self.assertIn("category=person", scene.text)
        self.assertEqual(SceneDescription.from_dict(scene.to_dict()), scene)

    def test_scene_environment_is_strict(self) -> None:
        environment = valid_environment().to_dict()
        for mutation, message in (
            ({**environment, "extra": True}, "extra is not allowed"),
            (
                {key: value for key, value in environment.items() if key != "lighting"},
                "lighting is required",
            ),
            ({**environment, "path_clearance": "mostly_clear"}, "must be one of"),
            ({**environment, "hazards": ["spill", "spill"]}, "unique values"),
        ):
            with self.subTest(mutation=mutation):
                with self.assertRaisesRegex(SchemaValidationError, message):
                    SceneEnvironment.from_dict(mutation)

    def test_scene_obstacle_is_strict_and_rejects_invalid_enum(self) -> None:
        valid = {
            "category": "box",
            "position": "front",
            "distance_level": "near",
            "motion": "static",
            "risk": "low",
        }
        for mutation, message in (
            ({**valid, "extra": True}, "extra is not allowed"),
            ({key: value for key, value in valid.items() if key != "risk"}, "risk is required"),
            ({**valid, "position": "behind"}, "must be one of"),
        ):
            with self.subTest(mutation=mutation):
                with self.assertRaisesRegex(SchemaValidationError, message):
                    SceneObstacle.from_dict(mutation)

    def test_legacy_obstacle_defaults_expanded_fields_to_unknown(self) -> None:
        obstacle = SceneObstacle.from_dict(
            {
                "category": "box",
                "position": "front",
                "distance_level": "near",
                "motion": "static",
                "risk": "low",
            }
        )
        self.assertEqual(obstacle.material, "unknown")
        self.assertEqual(obstacle.support_type, "unknown")
        self.assertEqual(obstacle.path_relevance, "unknown")
        self.assertEqual(len(obstacle.to_dict()), 11)

    def test_expanded_obstacle_requires_all_fields_and_strict_enums(self) -> None:
        expanded = {
            "category": "cardboard box",
            "position": "front",
            "distance_level": "near",
            "motion": "static",
            "risk": "low",
            "material": "cardboard",
            "support_type": "flat_base",
            "attachment_state": "unattached",
            "fragility": "non_fragile",
            "size_level": "medium",
            "path_relevance": "blocking",
        }
        self.assertEqual(SceneObstacle.from_dict(expanded).material, "cardboard")
        for mutation, message in (
            (
                {key: value for key, value in expanded.items() if key != "fragility"},
                "fragility is required",
            ),
            ({**expanded, "material": "paper"}, "must be one of"),
            ({**expanded, "path_relevance": "in_path"}, "must be one of"),
        ):
            with self.subTest(mutation=mutation):
                with self.assertRaisesRegex(SchemaValidationError, message):
                    SceneObstacle.from_dict(mutation)

    def test_structured_scene_rejects_empty_or_invalid_primary_index(self) -> None:
        with self.assertRaisesRegex(SchemaValidationError, "must not be empty"):
            SceneDescription.from_dict(
                {"text": "none", "obstacles": [], "primary_obstacle_index": 0}
            )

        scene = SceneDescription.from_obstacles(
            (SceneObstacle("box", "front", "near", "static", "low"),)
        ).to_dict()
        scene["primary_obstacle_index"] = 1
        with self.assertRaisesRegex(SchemaValidationError, "must index"):
            SceneDescription.from_dict(scene)

    def test_primary_selection_prioritizes_risk_distance_and_position(self) -> None:
        obstacles = (
            SceneObstacle("low", "front", "near", "static", "low"),
            SceneObstacle("far", "front", "far", "static", "high"),
            SceneObstacle("side", "left", "near", "static", "high"),
            SceneObstacle("front", "front", "near", "static", "high"),
            SceneObstacle("tie", "front", "near", "static", "high"),
        )
        self.assertEqual(select_primary_obstacle(obstacles), 0)

    def test_primary_selection_prioritizes_path_relevance_before_risk(self) -> None:
        obstacles = (
            SceneObstacle(
                "side hazard", "left", "near", "static", "high",
                path_relevance="near_path",
            ),
            SceneObstacle(
                "route box", "front", "far", "static", "low",
                path_relevance="blocking",
            ),
            SceneObstacle(
                "clear object", "front", "near", "static", "high",
                path_relevance="not_blocking",
            ),
        )
        self.assertEqual(select_primary_obstacle(obstacles), 0)


class OutputSchemaTests(unittest.TestCase):
    def test_shared_json_schema_is_strict(self) -> None:
        self.assertFalse(DECISION_JSON_SCHEMA["additionalProperties"])
        self.assertEqual(
            set(DECISION_JSON_SCHEMA["required"]),
            set(DECISION_JSON_SCHEMA["properties"]),
        )
        self.assertTrue(
            DECISION_JSON_SCHEMA["properties"]["risk_flags"]["uniqueItems"]
        )

    def test_distribution_is_normalized_and_probabilities_are_clamped(self) -> None:
        output = DecisionOutput(
            object_assessment=ObjectAssessment("box", 1.2),
            pushability_probability=-0.5,
            decision_distribution=DecisionDistribution(2.0, 1.0, 1.0),
            risk_flags=("unknown_mass", "unknown_mass"),
            uncertainty=2.0,
            reason="safe fallback",
        ).to_dict()
        self.assertEqual(output["object_assessment"]["confidence"], 1.0)
        self.assertEqual(output["pushability_probability"], 0.0)
        self.assertAlmostEqual(
            sum(output["decision_distribution"].values()),
            1.0,
            places=6,
        )
        self.assertEqual(output["risk_flags"], ["unknown_mass"])
        self.assertEqual(output["uncertainty"], 1.0)


if __name__ == "__main__":
    unittest.main()
