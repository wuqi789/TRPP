#!/usr/bin/env python3
"""ROS 2 node that turns user text into a NavigationIntent message."""

from __future__ import annotations

import rclpy
import uuid
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from semantic_navigation_interfaces.msg import NavigationIntent

from llm_agent.json_parser import NavigationIntentParseError, parse_navigation_intent
from llm_agent.llm_interface import LLMError, LLMInterface
from llm_agent.utils import load_yaml


class LLMAgentNode(Node):
    def __init__(self) -> None:
        super().__init__("llm_agent_node")
        share = get_package_share_directory("llm_module1")
        self.declare_parameter("config_path", f"{share}/config/config.yaml")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self._llm = LLMInterface(load_yaml(config_path))
        self._publisher = self.create_publisher(
            NavigationIntent, "/semantic_navigation/intent", 10
        )
        self._subscription = self.create_subscription(String, "/user_instruction", self._on_instruction, 10)
        self.get_logger().info(f"Semantic reasoner ready (mode={self._llm.mode})")

    def _on_instruction(self, message: String) -> None:
        try:
            intent = parse_navigation_intent(self._llm.infer(message.data))
        except LLMError as exc:
            self.get_logger().error(f"Instruction failed [{exc.code}]")
            return
        except (NavigationIntentParseError, ValueError):
            self.get_logger().error("Instruction failed [INVALID_INTENT_JSON]")
            return
        output = NavigationIntent()
        output.request_id = str(uuid.uuid4())
        output.source_text = message.data
        output.goal_object = intent.goal_object
        output.reference_object = intent.reference_object
        output.relation = intent.relation
        output.strategy = intent.strategy
        output.constraints = list(intent.constraints)
        self._publisher.publish(output)
        self.get_logger().info(
            f"Published navigation intent {output.request_id} for '{intent.goal_object}'"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LLMAgentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
