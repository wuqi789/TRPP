from pathlib import Path

import pytest

from matching import EntityMatch, HybridEntityMatcher
from providers import YamlSemanticMapProvider
from resolution import ResolutionError, SemanticResolver


def _constant_encoder(values):
    return [[1.0, 0.0] for _ in values]


@pytest.fixture()
def resolver():
    map_path = Path(__file__).parents[1] / "maps" / "DCT_hospital_demo.yaml"
    provider = YamlSemanticMapProvider(map_path, "jackal_robot")
    matcher = HybridEntityMatcher(provider.ontology, encoder=_constant_encoder)
    return SemanticResolver(provider, matcher)


def test_alias_then_relation_selects_expected_cracker_box(resolver):
    result = resolver.resolve(
        {
            "request_id": "request-1",
            "goal_object": "biscuit cartoncracker box",
            "reference_object": "medical supply trolley",
            "relation": "near",
            "constraints": [],
            "strategy": "shortest",
        }
    )

    assert result.goal_id == "cracker_box_01"
    assert result.match_method == "alias"
    assert result.pose == pytest.approx((2.786, 0.369, 0.647), abs=1e-6)


def test_instance_id_precedes_class_matching_and_constraints_are_executable(resolver):
    result = resolver.resolve(
        {
            "request_id": "request-2",
            "goal_object": "cracker_box_01",
            "constraints": ["via chair_01", "avoid trashcan_01"],
        }
    )

    assert result.match_method == "id"
    assert [value.type for value in result.constraints] == ["via", "avoid"]
    assert [value.entity_id for value in result.constraints] == [
        "chair_01",
        "trashcan_01",
    ]
    assert result.constraints[1].radius == pytest.approx(0.683)


def test_unsupported_constraint_is_explicit(resolver):
    with pytest.raises(ResolutionError) as raised:
        resolver.resolve(
            {
                "request_id": "request-3",
                "goal_object": "chair",
                "constraints": ["quietly"],
            }
        )
    assert raised.value.code == "UNSUPPORTED_CONSTRAINT"


def test_embedding_threshold_and_margin_are_enforced():
    def encoder(values):
        vectors = {
            "alpha": [1.0, 0.0],
            "beta": [0.0, 1.0],
            "unclear": [0.71, 0.70],
            "weak": [-1.0, -1.0],
        }
        return [vectors[value] for value in values]

    matcher = HybridEntityMatcher(
        {"alpha": [], "beta": []},
        encoder=encoder,
        minimum_similarity=0.62,
        minimum_margin=0.08,
    )
    with pytest.raises(ValueError, match="AMBIGUOUS"):
        matcher.match("unclear")
    with pytest.raises(ValueError, match="TOO_LOW"):
        matcher.match("weak")


def test_exact_match_does_not_load_embedding_runtime(monkeypatch):
    matcher = HybridEntityMatcher({"sofa": ["couch", "\u6c99\u53d1"]})

    def fail_if_loaded():
        raise AssertionError("embedding runtime must remain lazy for exact ontology matches")

    monkeypatch.setattr(matcher, "_load_encoder", fail_if_loaded)
    assert matcher.match("sofa") == EntityMatch("sofa", "canonical", 1.0)
    assert matcher.match("couch") == EntityMatch("sofa", "alias", 1.0)


def test_missing_embedding_runtime_is_a_typed_resolution_failure(resolver, monkeypatch):
    def unavailable():
        raise RuntimeError("semantic model is unavailable")

    monkeypatch.setattr(resolver.matcher, "_load_encoder", unavailable)
    with pytest.raises(ResolutionError) as raised:
        resolver.resolve(
            {
                "request_id": "request-4",
                "goal_object": "chair_01",
                "constraints": ["via unknown landmark"],
            }
        )
    assert raised.value.code == "SEMANTIC_MATCHER_UNAVAILABLE"
