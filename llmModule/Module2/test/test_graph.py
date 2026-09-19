from pathlib import Path

from builder import build_graph, load_map
from query import (
    GroundingError,
    ground_entity,
    query_entities,
    query_relation,
    query_topology,
    relative_pose,
)
import pytest


MAP_PATH = Path(__file__).parents[1] / "maps" / "DCT_hospital_demo.yaml"


def make_graph():
    return build_graph(load_map(MAP_PATH))


def test_loads_hospital_map_and_builds_graph():
    graph = make_graph()
    assert graph.get_node("hospital_scene") is not None
    assert graph.get_node("jackal_robot") is not None


def test_hospital_regression_fixture_is_self_contained():
    data = load_map(MAP_PATH)

    assert MAP_PATH.is_file()
    assert data["rooms"][0]["id"] == "hospital_scene"


def test_relation_disambiguates_cracker_boxes():
    graph = make_graph()
    near_cart = ground_entity(graph, "cracker_box", "supply_cart", "near")
    near_disposal = ground_entity(graph, "cracker_box", "disposal_stand", "near")

    assert near_cart.id == "cracker_box_01"
    assert (near_cart.position.x, near_cart.position.y, near_cart.position.theta) == (
        -7.214,
        -0.631,
        0.647,
    )
    assert near_disposal.id == "cracker_box_02"


def test_cracker_box_pose_is_converted_from_stage_to_odom():
    graph = make_graph()
    target = graph.get_node("cracker_box_01")
    origin = graph.get_node("jackal_robot")
    x, y, theta = relative_pose(target.position, origin.position)
    assert (round(x, 3), round(y, 3), round(theta, 3)) == (2.786, 0.369, 0.647)


def test_ambiguous_goal_without_relation_is_rejected():
    with pytest.raises(GroundingError, match="ambiguous"):
        ground_entity(make_graph(), "cracker_box")


def test_unique_goal_does_not_require_relation():
    assert ground_entity(make_graph(), "hospital_bed").id == "hospital_bed_01"


def test_queries_topology_path():
    path = query_topology(make_graph(), "jackal_robot", "cracker_box_01")
    assert path == ["jackal_robot", "hospital_scene", "cracker_box_01"]
