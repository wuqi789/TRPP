#!/usr/bin/env python3
"""ROS adapter for replaceable semantic-map grounding."""

from __future__ import annotations

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from llm_module2.srv import GetPose, QueryEntity, QueryRelation, QueryTopology
from matching import HybridEntityMatcher
from providers import YamlSemanticMapProvider
from query import query_entities, query_relation
from resolution import ResolutionError, SemanticResolver
from semantic_navigation_interfaces.msg import (
    Constraint,
    NavigationIntent,
    ResolutionResult,
)


class SemanticGraphNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_graph_node")
        share = get_package_share_directory("llm_module2")
        self.declare_parameter("map_path", f"{share}/maps/scout_kujiale_0021.yaml")
        self.declare_parameter("navigation_origin_id", "jackal_robot")
        self.declare_parameter("minimum_similarity", 0.62)
        self.declare_parameter("minimum_margin", 0.08)
        map_path = str(self.get_parameter("map_path").value)
        origin_id = str(self.get_parameter("navigation_origin_id").value)
        self._provider = YamlSemanticMapProvider(map_path, origin_id)
        self._matcher = HybridEntityMatcher(
            self._provider.ontology,
            minimum_similarity=float(self.get_parameter("minimum_similarity").value),
            minimum_margin=float(self.get_parameter("minimum_margin").value),
        )
        self._resolver = SemanticResolver(self._provider, self._matcher)
        self._graph = self._provider.graph

        self.create_service(QueryEntity, "/semantic_graph/query_entity", self._query_entity)
        self.create_service(QueryRelation, "/semantic_graph/query_relation", self._query_relation)
        self.create_service(GetPose, "/semantic_graph/get_pose", self._get_pose)
        self.create_service(QueryTopology, "/semantic_graph/query_topology", self._query_topology)

        self._publisher = self.create_publisher(
            ResolutionResult, "/semantic_navigation/resolution", 10
        )
        self.create_subscription(
            NavigationIntent,
            "/semantic_navigation/intent",
            self._on_navigation_intent,
            10,
        )
        self.get_logger().info(
            f"Loaded semantic map revision={self._provider.revision[:12]}"
        )

    def _query_entity(self, request, response):
        matches = query_entities(self._graph, request.entity_name)
        response.success = bool(matches)
        response.ids = [node.id for node in matches]
        poses = [self._provider.navigation_pose(node) for node in matches]
        response.x = [pose[0] for pose in poses]
        response.y = [pose[1] for pose in poses]
        return response

    def _query_relation(self, request, response):
        try:
            matches = query_relation(self._graph, request.source, request.relation, request.target)
        except ValueError:
            self.get_logger().warning("Semantic relation query rejected")
            matches = []
        response.success = bool(matches)
        response.matched_id = matches[0].id if matches else ""
        return response

    def _get_pose(self, request, response):
        node = self._graph.get_node(request.object_id)
        response.success = node is not None
        if node is not None:
            response.x, response.y, response.theta = self._provider.navigation_pose(node)
        return response

    def _query_topology(self, request, response):
        response.path = self._provider.topology_path(request.start, request.goal)
        response.success = bool(response.path)
        return response

    def _on_navigation_intent(self, intent: NavigationIntent) -> None:
        self._resolve_and_publish(intent)

    def _resolve_and_publish(self, intent: NavigationIntent) -> None:
        result = ResolutionResult()
        result.intent = intent
        result.request_id = intent.request_id
        result.map_revision = self._provider.revision
        result.strategy = intent.strategy or "shortest"
        try:
            resolved = self._resolver.resolve(
                {
                    "request_id": intent.request_id,
                    "goal_object": intent.goal_object,
                    "reference_object": intent.reference_object,
                    "relation": intent.relation,
                    "constraints": list(intent.constraints),
                    "strategy": intent.strategy,
                }
            )
        except ResolutionError as exc:
            result.resolved = False
            result.error_code = exc.code
            result.message = exc.code
            self._publisher.publish(result)
            self.get_logger().warning(
                f"Resolution {intent.request_id} failed: {exc.code}"
            )
            return

        result.resolved = True
        result.goal_id = resolved.goal_id
        result.canonical_name = resolved.canonical_name
        result.match_method = resolved.match_method
        result.similarity = resolved.similarity
        result.goal_pose = self._pose_stamped(resolved.pose)
        result.topology_context = list(resolved.topology_context)
        result.constraints = [self._constraint_message(value) for value in resolved.constraints]
        result.message = "Semantic goal resolved"
        self._publisher.publish(result)
        self.get_logger().info(
            f"Resolved {intent.request_id} to {resolved.goal_id} "
            f"({resolved.match_method}, {resolved.similarity:.3f})"
        )

    def _pose_stamped(self, pose: tuple[float, float, float]) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = "odom"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose[0]
        message.pose.position.y = pose[1]
        message.pose.orientation.z = __import__("math").sin(pose[2] / 2.0)
        message.pose.orientation.w = __import__("math").cos(pose[2] / 2.0)
        return message

    def _constraint_message(self, value) -> Constraint:
        message = Constraint()
        message.type = value.type
        message.source_text = value.source_text
        message.entity_id = value.entity_id
        message.canonical_name = value.canonical_name
        message.pose = self._pose_stamped(value.pose)
        message.radius = value.radius
        message.order = value.order
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
