from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_static_verification_defaults_are_fail_closed():
    config = yaml.safe_load((ROOT / "config" / "verification.yaml").read_text())

    assert config["entity_check"]["enabled"] is True
    assert config["topology_check"]["enabled"] is True
    assert config["geometry_check"]["enabled"] is True
    assert config["planner_check"]["enabled"] is True
    assert config["dynamic_check"]["enabled"] is False
    assert config["validation"]["planner_namespace"] == "semantic_validation"
    assert config["validation"]["global_frame"] == "odom"
    assert config["validation"]["robot_frame"] == "base_link"


def test_validation_planner_uses_scout_map_and_navfn_astar():
    config = yaml.safe_load(
        (ROOT / "config" / "nav2_validation_astar.yaml").read_text()
    )
    planner = config["/semantic_validation/planner_server"]["ros__parameters"]
    costmap = config[
        "/semantic_validation/global_costmap/global_costmap"
    ]["ros__parameters"]

    assert planner["GridBased"]["plugin"] == "nav2_navfn_planner/NavfnPlanner"
    assert planner["GridBased"]["use_astar"] is True
    assert planner["GridBased"]["allow_unknown"] is False
    assert costmap["global_frame"] == "odom"
    assert costmap["robot_base_frame"] == "base_link"
    assert costmap["static_layer"]["map_topic"] == "/semantic_validation/map"
    assert costmap["keepout_filter"]["filter_info_topic"] == (
        "/semantic_validation/keepout_filter_info"
    )
    assert "0.31" in costmap["footprint"]
    assert "0.2925" in costmap["footprint"]


def test_module1_to_module3_launch_excludes_execution_layer():
    launch_path = ROOT / "launch" / "module1_to_module3.launch.py"
    source = launch_path.read_text()
    compile(source, str(launch_path), "exec")

    assert '"llm_module1"' in source
    assert '"llm_module2"' in source
    assert '"llm_module3"' in source
    assert '"validation_planner.launch.py"' in source
    assert "llm_module4" not in source
    assert "semantic_navigation_compatibility" not in source
    assert "/semantic_navigation/execute" not in source
