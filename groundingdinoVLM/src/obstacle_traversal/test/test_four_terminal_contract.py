"""Public contract checks for the Isaac traversal workspace.

These tests intentionally verify names and provider boundaries only.  Private
model prompts, thresholds, weights and decision rules are not part of the
published test surface.
"""

from pathlib import Path


WORKSPACE = next(
    parent for parent in Path(__file__).resolve().parents
    if (parent / "groundingdinoVLM").is_dir()
)


def read(relative: str) -> str:
    return (WORKSPACE / relative).read_text(encoding="utf-8")


def test_public_launch_uses_only_local_mock_nodes():
    source = read("groundingdinoVLM/src/obstacle_traversal/launch/public_isaac_traversal.launch.py")
    for executable in (
        "mock_verify_target", "mock_assess_pushability", "mock_probe_pushability",
        "mock_approach_obstacle", "mock_yolo_detector", "mock_piper_arm", "fixed_route_driver",
    ):
        assert f'executable="{executable}"' in source
    assert "llm_module4" not in source
    assert "yolo_model" not in source


def test_public_topics_and_actions_are_explicit():
    source = read("groundingdinoVLM/src/scout_public_mock/scout_public_mock/mock_nodes.py")
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


def test_public_route_driver_waits_for_localization_and_reports_terminal_states():
    source = read("groundingdinoVLM/src/scout_public_mock/scout_public_mock/mock_nodes.py")
    assert "WAITING_FOR_ODOM" in source
    assert "PUBLIC_ROUTE_TIMEOUT" in source
    assert "goal_x" in source and "goal_y" in source


def test_default_target_verifier_is_mock_and_external_is_a_narrow_adapter():
    mock_config = read("groundingdinoVLM/src/groundingdino_vlm/config/mock.yaml")
    external_config = read("groundingdinoVLM/src/groundingdino_vlm/config/external.yaml")
    detector = read("groundingdinoVLM/src/groundingdino_vlm/groundingdino_vlm/detectors.py")
    verifier = read("groundingdinoVLM/src/groundingdino_vlm/groundingdino_vlm/vlm.py")
    assert "provider: mock" in mock_config
    assert "provider: external" in external_config
    assert "SCOUT_TARGET_DETECTOR_ADAPTER" in detector
    assert "SCOUT_TARGET_VLM_ADAPTER" in verifier
    assert "module:factory" in detector and "module:factory" in verifier


def test_yolo_boundary_has_no_model_path_or_local_inference_dependency():
    source = read("groundingdinoVLM/src/piper_probe/piper_probe/yolo_obstacle_detector.py")
    assert "SCOUT_OBSTACLE_DETECTOR_ADAPTER" in source
    assert "ultralytics" not in source
    assert "model_path" not in source
    assert "weights" not in source


def test_real_launch_uses_external_contract_without_private_paths():
    launch = read("groundingdinoVLM/src/obstacle_traversal/launch/real_isaac_traversal.launch.py")
    assert "external.yaml" in launch
    assert "yolo_model" not in launch
    assert "groundingdinoVLM/.deps" not in launch
    assert "wall_curtain_yolov8" not in launch


def test_scripts_are_noninteractive_and_path_relative():
    script = read("run_traversal_system.sh")
    assert 'SCOUT_PUBLIC_BACKEND="${SCOUT_PUBLIC_BACKEND:-mock}"' in script
    assert "read -" + "rsp" not in script
    assert '$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")"' in script


def test_public_sources_do_not_embed_provider_transport_or_credentials():
    roots = (
        "groundingdinoVLM/src/groundingdino_vlm",
        "groundingdinoVLM/src/piper_probe",
        "llmdecision",
        "llmModule",
    )
    forbidden = (
        "Authorization: " + "Bearer", "/chat" + "/completions", "paramiko",
        "open_" + "sftp", "private_" + "key", "BEGIN " + "RSA PRIVATE KEY",
        "TEAM" + "AIHUB", "DASH" + "SCOPE",
    )
    for root in roots:
        for path in (WORKSPACE / root).rglob("*"):
            if path.is_file() and path.suffix not in {".pyc", ".png", ".jpg"}:
                text = path.read_text(encoding="utf-8", errors="ignore")
                assert not any(marker in text for marker in forbidden), path
