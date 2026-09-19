from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_scout_full_chain_launch_composes_module1_to_module4():
    source = (
        PACKAGE / "launch" / "scout_semantic_navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert '"module1_to_module3.launch.py"' in source
    assert '"navigation.launch.py"' in source
    assert '"affordance.launch.py"' in source
    assert '"use_pushability_mapping"' in source
    assert '"module3_verification_config_path"' in source
    assert '"verification_config_path"' in source
    assert "dct_isaac_ros2" not in source
