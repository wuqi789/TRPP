from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def test_module4_defaults_to_local_route_mock():
    config = yaml.safe_load((PACKAGE / "config/navigation.yaml").read_text())
    assert config["vlm"]["mode"] == "mock"
    assert config["vlm"]["adapter_env"] == "SCOUT_ROUTE_ADAPTER"
    assert config["topics"]["goal"] == "/clicked_point"
