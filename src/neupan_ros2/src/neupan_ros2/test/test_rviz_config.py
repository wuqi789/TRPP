from pathlib import Path


def test_isaac_rviz_uses_live_rgb_instead_of_frozen_semantic_overlay():
    config = (
        Path(__file__).parents[1] / "rviz" / "isaac_sim.rviz"
    ).read_text(encoding="utf-8")

    assert "Name: Semantic mapping input" in config
    assert "Value: /isaac/color_image_raw" in config
    assert "Value: /semantic_mapping/overlay" not in config
