from pathlib import Path
import threading
import time

import numpy as np
import pytest

try:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, String

    from groundingdino_vlm.node import GroundingDinoVLMNode
    from groundingdino_vlm_interfaces.msg import TargetVerification
except ImportError:
    pytest.skip("ROS 2 Python packages are unavailable", allow_module_level=True)


def transient_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def image_message(node: Node) -> Image:
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[30:90, 40:120] = (100, 160, 220)
    output = Image()
    output.header.stamp = node.get_clock().now().to_msg()
    output.height, output.width = image.shape[:2]
    output.encoding = "bgr8"
    output.is_bigendian = False
    output.step = output.width * 3
    output.data = image.tobytes()
    return output


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_mock_ros_topics_reach_verified_then_reset_to_idle(tmp_path, monkeypatch):
    ros_log_dir = tmp_path / "ros-log"
    ros_log_dir.mkdir()
    monkeypatch.setenv("ROS_LOG_DIR", str(ros_log_dir))
    rclpy.init()
    verifier = None
    probe = None
    executor = None
    spin_thread = None
    try:
        config = Path(get_package_share_directory("groundingdino_vlm")) / "config" / "mock.yaml"
        verifier = GroundingDinoVLMNode(config_override=str(config))
        probe = Node("groundingdino_vlm_test_probe")
        labels = probe.create_publisher(
            String, "/groundingdino_vlm/target_label", transient_qos()
        )
        images = probe.create_publisher(
            Image, "/isaac/color_image_raw", qos_profile_sensor_data
        )
        results = []
        readiness = []
        debug_images = []
        probe.create_subscription(
            TargetVerification,
            "/groundingdino_vlm/result",
            results.append,
            transient_qos(),
        )
        probe.create_subscription(
            Bool,
            "/groundingdino_vlm/ready",
            lambda message: readiness.append(message.data),
            transient_qos(),
        )
        probe.create_subscription(
            Image,
            "/groundingdino_vlm/debug_image",
            debug_images.append,
            qos_profile_sensor_data,
        )
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(verifier)
        executor.add_node(probe)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        assert wait_for(lambda: labels.get_subscription_count() == 1)

        labels.publish(String(data="chair"))
        for _ in range(7):
            images.publish(image_message(probe))
            time.sleep(0.08)

        assert wait_for(lambda: any(message.verified for message in results))
        verified = next(message for message in reversed(results) if message.verified)
        assert verified.state == TargetVerification.VERIFIED
        assert verified.target_label == "chair"
        assert verified.vlm_provider == "mock"
        assert verified.candidates[0].phrase == "chair"
        assert verified.candidates[0].grounding_confidence == pytest.approx(0.95)
        assert verified.dino_latency_ms >= 0.0
        assert verified.vlm_latency_ms >= 0.0
        assert debug_images and debug_images[-1].data
        assert any(readiness)

        late_results = []
        probe.create_subscription(
            TargetVerification,
            "/groundingdino_vlm/result",
            late_results.append,
            transient_qos(),
        )
        assert wait_for(lambda: bool(late_results))
        assert late_results[-1].verified

        previous_revision = verified.target_revision
        labels.publish(String(data="table"))
        assert wait_for(
            lambda: bool(results)
            and results[-1].target_label == "table"
            and results[-1].target_revision > previous_revision
        )
        assert results[-1].state == TargetVerification.DETECTING
        assert not results[-1].verified

        labels.publish(String(data=""))
        assert wait_for(
            lambda: bool(results) and results[-1].state == TargetVerification.IDLE
        )
        assert not results[-1].verified
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if verifier is not None:
            verifier.destroy_node()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


@pytest.mark.parametrize(
    "decision,error_code,expected_state,expected_error",
    [
        ("REJECT", "", TargetVerification.REJECTED, ""),
        ("ACCEPT", "VLM_AUTHENTICATION_FAILED", TargetVerification.ERROR,
         "VLM_AUTHENTICATION_FAILED"),
    ],
)
def test_mock_ros_vlm_reject_and_error_fail_closed(
    tmp_path,
    monkeypatch,
    decision,
    error_code,
    expected_state,
    expected_error,
):
    ros_log_dir = tmp_path / "ros-log"
    ros_log_dir.mkdir()
    monkeypatch.setenv("ROS_LOG_DIR", str(ros_log_dir))
    config = tmp_path / "mock-case.yaml"
    config.write_text(
        f"""detector:
  provider: mock
  detected: true
  confidence: 0.91
vlm:
  provider: mock
  decision: {decision}
  error_code: {error_code!r}
temporal:
  window_size: 3
  required_positive: 2
runtime:
  maximum_inference_rate: 20.0
  image_freshness_timeout: 2.0
""",
        encoding="utf-8",
    )

    rclpy.init()
    verifier = None
    probe = None
    executor = None
    spin_thread = None
    try:
        verifier = GroundingDinoVLMNode(config_override=str(config))
        probe = Node("groundingdino_vlm_failure_probe")
        labels = probe.create_publisher(
            String, "/groundingdino_vlm/target_label", transient_qos()
        )
        images = probe.create_publisher(
            Image, "/isaac/color_image_raw", qos_profile_sensor_data
        )
        results = []
        probe.create_subscription(
            TargetVerification,
            "/groundingdino_vlm/result",
            results.append,
            transient_qos(),
        )
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(verifier)
        executor.add_node(probe)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        assert wait_for(lambda: labels.get_subscription_count() == 1)

        labels.publish(String(data="chair"))
        images.publish(image_message(probe))
        assert wait_for(lambda: any(item.state == expected_state for item in results))
        terminal = next(item for item in reversed(results) if item.state == expected_state)
        assert not terminal.verified
        assert terminal.error_code == expected_error

        for _ in range(3):
            images.publish(image_message(probe))
            time.sleep(0.06)
        assert not any(item.verified for item in results)
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if verifier is not None:
            verifier.destroy_node()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
