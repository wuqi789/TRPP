from pathlib import Path

import pytest

from builder import load_map
from matching import HybridEntityMatcher
from providers import YamlSemanticMapProvider
from resolution import SemanticResolver


MAP_PATH = Path(__file__).parents[1] / "maps" / "scout_kujiale_0021.yaml"


def test_scout_map_contract_and_counts():
    data = load_map(MAP_PATH)
    provider = YamlSemanticMapProvider(MAP_PATH, "jackal_robot")

    assert len(provider.graph.nodes()) == 34
    assert len(data["relations"]) == 52
    assert len(data["rooms"]) == 5
    assert data["corridors"] == []
    assert provider.entity("jackal_robot").name == "scout_mini"


def test_all_rooms_are_topologically_reachable():
    provider = YamlSemanticMapProvider(MAP_PATH, "jackal_robot")

    for room_id in ("living_room", "bedroom", "balcony", "kitchen", "bathroom"):
        path = provider.topology_path("jackal_robot", room_id)
        assert path[0] == "jackal_robot"
        assert path[-1] == room_id


def test_refrigerator_navigation_pose_uses_scout_origin():
    provider = YamlSemanticMapProvider(MAP_PATH, "jackal_robot")
    pose = provider.navigation_pose(provider.entity("kitchen_fridge_0000"))

    assert pose == pytest.approx((4.6243, -3.2823, -1.0), abs=1e-4)


@pytest.mark.parametrize("label", ["curtain", "fabric curtain", "门帘"])
def test_scout_curtain_constraint_resolves_without_embedding_runtime(label, monkeypatch):
    provider = YamlSemanticMapProvider(MAP_PATH, "jackal_robot")
    matcher = HybridEntityMatcher(provider.ontology)

    def fail_if_loaded():
        raise AssertionError("curtain landmark must resolve from map properties")

    monkeypatch.setattr(matcher, "_load_encoder", fail_if_loaded)
    result = SemanticResolver(provider, matcher).resolve(
        {
            "request_id": "curtain-task",
            "goal_object": "bathroom",
            "constraints": [f"via {label}"],
        }
    )

    assert result.goal_id == "bathroom"
    assert len(result.constraints) == 1
    assert result.constraints[0].entity_id == "doorway_0000"
    assert result.constraints[0].canonical_name == "doorway"
