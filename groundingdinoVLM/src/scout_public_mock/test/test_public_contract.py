from pathlib import Path


ROOT = Path(__file__).parents[4]
MOCK = ROOT / "groundingdinoVLM/src/scout_public_mock/scout_public_mock/mock_nodes.py"
LAUNCH = ROOT / "groundingdinoVLM/src/obstacle_traversal/launch/public_isaac_traversal.launch.py"


def test_public_launch_does_not_load_private_providers():
    source = LAUNCH.read_text(encoding="utf-8")
    assert "public_isaac_traversal.launch.py" not in source
    for executable in (
        "mock_verify_target", "mock_assess_pushability", "mock_probe_pushability",
        "mock_approach_obstacle", "mock_yolo_detector", "mock_piper_arm", "fixed_route_driver",
    ):
        assert f'executable="{executable}"' in source
    assert "llm_module4" not in source
    assert "yolo_model" not in source


def test_ros_contracts_and_fixed_route_are_explicit():
    source = MOCK.read_text(encoding="utf-8")
    for action in (
        "/groundingdino_vlm/verify_target",
        "/llmdecision/assess_pushability",
        "/piper/probe_pushability",
        "/obstacle_traversal/approach_obstacle",
    ):
        assert action in source
    for topic in (
        "/piper/yolo_ready", "/piper/yolo_obstacle_result",
        "/isaac_joint_states", "/isaac_joint_command", "/clicked_point", "/odom",
        "/obstacle_traversal/status",
    ):
        assert topic in source
    assert "4.20" in source
    assert "WAITING_FOR_ODOM" in source
    assert "PUBLIC_ROUTE_TIMEOUT" in source


def test_start_script_is_noninteractive_by_default():
    script = (ROOT / "run_traversal_system.sh").read_text(encoding="utf-8")
    assert 'SCOUT_PUBLIC_BACKEND="${SCOUT_PUBLIC_BACKEND:-mock}"' in script
    assert "read -" + "rsp" not in script
    assert "SCOUT_PUBLIC_BACKEND=real" in script
