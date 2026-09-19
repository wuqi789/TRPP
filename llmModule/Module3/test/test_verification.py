import math
import sys
from pathlib import Path


MODULE3_ROOT = Path(__file__).resolve().parents[1]
MODULE2_ROOT = MODULE3_ROOT.parent / "Module2"
HOSPITAL_MAP_PATH = MODULE2_ROOT / "maps" / "DCT_hospital_demo.yaml"

# Import Module2's source packages so this test exercises its real YAML loader
# and graph builder without requiring an installed ROS 2 workspace.
sys.path.insert(0, str(MODULE2_ROOT))

from builder import build_graph, load_map
from verification_core import NavigationVerifier, OccupancyGrid


def make_graph():
    return build_graph(load_map(HOSPITAL_MAP_PATH))


def make_verifier(occupied=(), start_node="jackal_robot"):
    config = {
        "entity_check": {"enabled": True},
        "topology_check": {"enabled": True},
        "geometry_check": {"enabled": True},
        "dynamic_check": {"enabled": True},
    }
    grid = OccupancyGrid(
        22,
        12,
        resolution=1.0,
        origin_x=-2.0,
        origin_y=-4.0,
        occupied=frozenset(occupied),
    )
    return NavigationVerifier(config, grid, start_node=start_node)


def cracker_box_odom_pose():
    return {"x": 2.786, "y": 0.369, "theta": 0.647}


def test_real_hospital_map_is_loaded():
    graph = make_graph()
    cracker_box = graph.get_node("cracker_box_01")

    assert cracker_box is not None
    assert cracker_box.name == "cracker_box"
    assert cracker_box.position.x == -7.214
    assert cracker_box.position.y == -0.631
    assert graph.has_relation("cracker_box_01", "near", "supply_cart_01")


def test_existing_goal_is_verified():
    graph = make_graph()
    position = cracker_box_odom_pose()
    result = make_verifier().verify_navigation(
        {"goal_id": "cracker_box_01", "position": position}, graph
    )
    assert result.verified is True
    assert result.goal_id == "cracker_box_01"
    assert result.failed_checks == ()


def test_unknown_goal_fails_entity_check():
    result = make_verifier().verify_navigation(
        {"goal_id": "unknown_object", "position": {"x": 1.0, "y": 1.0}}, make_graph()
    )
    assert result.verified is False
    assert list(result.failed_checks) == ["entity_not_found"]


def test_unreachable_goal_fails_topology_check():
    result = make_verifier().verify_navigation(
        {"goal_id": "route_goal", "position": {"x": 7.9, "y": 5.0}}, make_graph()
    )
    assert result.verified is False
    assert list(result.failed_checks) == ["topology_unreachable"]


def test_out_of_bounds_goal_fails_geometry_check():
    graph = make_graph()
    result = make_verifier().verify_navigation(
        {"goal_id": "cracker_box_01", "position": {"x": -20.0, "y": -0.631}}, graph
    )
    assert result.verified is False
    assert list(result.failed_checks) == ["geometry_invalid"]


def test_occupied_negative_coordinate_goal_fails_geometry_check():
    graph = make_graph()
    position = cracker_box_odom_pose()
    occupied_cell = (
        math.floor(position["x"] - -2.0),
        math.floor(position["y"] - -4.0),
    )
    result = make_verifier(occupied={occupied_cell}).verify_navigation(
        {"goal_id": "cracker_box_01", "position": position}, graph
    )
    assert result.verified is False
    assert list(result.failed_checks) == ["geometry_invalid"]

