from pathlib import Path

import yaml


def test_public_probe_config_contains_only_provider_boundary():
    config_path = Path(__file__).parents[1] / "config" / "isaac.yaml"
    params = yaml.safe_load(config_path.read_text(encoding="utf-8"))["piper_probe_server"]["ros__parameters"]
    assert params["provider"] == "external"
    assert params["adapter_env"] == "SCOUT_MECHANICAL_PROBE_ADAPTER"
    assert "effort_delta_thresholds" not in params
    assert "initial_pose" not in params
    assert "extend_pose" not in params
