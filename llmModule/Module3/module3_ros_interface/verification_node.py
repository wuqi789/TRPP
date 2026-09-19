#!/usr/bin/env python3
"""ROS adapter for four-stage semantic navigation verification."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from llm_module3.srv import VerifyNavigation
from providers import YamlSemanticMapProvider
from semantic_navigation_adapters import (
    CostmapCache,
    KeepoutMaskManager,
    Nav2PlannerClient,
    TfReadiness,
)
from semantic_navigation_interfaces.msg import (
    CheckResult,
    ResolutionResult,
    SystemReadiness,
    VerificationResult,
)
from verification_core import VerificationPipeline


class VerificationNode(Node):
    def __init__(self) -> None:
        super().__init__("verification_node")
        module2_share = get_package_share_directory("llm_module2")
        module3_share = get_package_share_directory("llm_module3")
        self.declare_parameter(
            "config_path", f"{module3_share}/config/verification.yaml"
        )
        with open(
            str(self.get_parameter("config_path").value), "r", encoding="utf-8"
        ) as stream:
            config = yaml.safe_load(stream) or {}
        validation = config.get("validation", {})
        entity_check = config.get("entity_check", {})
        topology_check = config.get("topology_check", {})
        geometry_check = config.get("geometry_check", {})
        planner_check = config.get("planner_check", {})
        for name, section in (
            ("entity_check_enabled", entity_check),
            ("topology_check_enabled", topology_check),
            ("geometry_check_enabled", geometry_check),
            ("planner_check_enabled", planner_check),
        ):
            self.declare_parameter(
                name,
                bool(section.get("enabled", True)),
                ParameterDescriptor(
                    read_only=True,
                    description=f"Startup-only Module3 research valve: {name}",
                ),
            )
        self.declare_parameter(
            "semantic_map_path", f"{module2_share}/maps/scout_kujiale_0021.yaml"
        )
        self.declare_parameter(
            "validation_namespace",
            str(validation.get("planner_namespace", "semantic_validation")),
        )
        self.declare_parameter(
            "validation_costmap_topic",
            str(
                validation.get(
                    "costmap_topic",
                    "/semantic_validation/global_costmap/costmap",
                )
            ),
        )
        self.declare_parameter(
            "validation_map_topic",
            str(validation.get("map_topic", "/semantic_validation/map")),
        )
        self.declare_parameter(
            "planner_timeout", float(validation.get("planner_timeout", 10.0))
        )
        self.declare_parameter(
            "ready_timeout", float(validation.get("ready_timeout", 30.0))
        )
        self.declare_parameter(
            "mask_update_timeout",
            float(validation.get("mask_update_timeout", 10.0)),
        )
        self.declare_parameter(
            "costmap_maximum_age",
            float(validation.get("costmap_maximum_age", 2.0)),
        )
        self.declare_parameter(
            "clock_stall_timeout",
            float(validation.get("clock_stall_timeout", 5.0)),
        )
        self.declare_parameter(
            "global_frame", str(validation.get("global_frame", "odom"))
        )
        self.declare_parameter(
            "robot_frame", str(validation.get("robot_frame", "base_link"))
        )
        self.declare_parameter(
            "robot_radius", float(validation.get("robot_radius", 0.333))
        )

        self._repository = YamlSemanticMapProvider(
            str(self.get_parameter("semantic_map_path").value), "jackal_robot"
        )
        namespace = str(self.get_parameter("validation_namespace").value).strip("/")
        self._geometry = CostmapCache(
            self,
            str(self.get_parameter("validation_costmap_topic").value),
            maximum_age=float(self.get_parameter("costmap_maximum_age").value),
            clock_stall_timeout=float(
                self.get_parameter("clock_stall_timeout").value
            ),
            occupied_threshold=99,
            robot_radius=float(self.get_parameter("robot_radius").value),
        )
        self._planner = Nav2PlannerClient(self, namespace=namespace)
        self._mask = KeepoutMaskManager(
            self,
            map_topic=str(self.get_parameter("validation_map_topic").value),
            mask_topic=f"/{namespace}/keepout_mask",
            info_topic=f"/{namespace}/keepout_filter_info",
        )
        self._tf = TfReadiness(
            self,
            global_frame=str(self.get_parameter("global_frame").value),
            robot_frame=str(self.get_parameter("robot_frame").value),
        )
        self._ready_timeout = float(self.get_parameter("ready_timeout").value)
        self._pipeline = VerificationPipeline(
            self._repository,
            self._geometry,
            self._planner,
            self._mask,
            planner_timeout=float(self.get_parameter("planner_timeout").value),
            mask_update_timeout=float(
                self.get_parameter("mask_update_timeout").value
            ),
            entity_check_enabled=bool(
                self.get_parameter("entity_check_enabled").value
            ),
            topology_check_enabled=bool(
                self.get_parameter("topology_check_enabled").value
            ),
            geometry_check_enabled=bool(
                self.get_parameter("geometry_check_enabled").value
            ),
            planner_check_enabled=bool(
                self.get_parameter("planner_check_enabled").value
            ),
        )
        self._workers = ThreadPoolExecutor(max_workers=1, thread_name_prefix="verification")
        self._publisher = self.create_publisher(
            VerificationResult, "/semantic_navigation/verification", 10
        )
        readiness_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._readiness_publisher = self.create_publisher(
            SystemReadiness,
            "/semantic_navigation/verification_readiness",
            readiness_qos,
        )
        self._readiness_state = None
        self._readiness_timer = self.create_timer(1.0, self._publish_readiness)
        self.create_subscription(
            ResolutionResult,
            "/semantic_navigation/resolution",
            self._on_resolution,
            10,
        )
        self._service = self.create_service(
            VerifyNavigation, "/verification/verify_navigation", self._legacy_service
        )
        gate_modes = ", ".join(
            f"{name.removesuffix('_check_enabled')}="
            f"{'enabled' if bool(self.get_parameter(name).value) else 'BYPASSED'}"
            for name in (
                "entity_check_enabled", "topology_check_enabled",
                "geometry_check_enabled", "planner_check_enabled",
            )
        )
        self.get_logger().info(
            f"Verification ready with isolated planner namespace '/{namespace}'; "
            f"research valves: {gate_modes}"
        )

    def _publish_readiness(self) -> None:
        namespace = str(self.get_parameter("validation_namespace").value).strip("/")
        geometry_enabled = bool(
            self.get_parameter("geometry_check_enabled").value
        )
        planner_enabled = bool(
            self.get_parameter("planner_check_enabled").value
        )
        costmap_status = self._geometry.status
        map_ready = self._mask.ready and bool(
            self.get_publishers_info_by_topic(
                str(self.get_parameter("validation_map_topic").value)
            )
        )
        keepout_ready = bool(
            self.get_subscriptions_info_by_topic(
                f"/{namespace}/keepout_filter_info"
            )
        ) and bool(
            self.get_subscriptions_info_by_topic(f"/{namespace}/keepout_mask")
        )
        values = [
            ("map", map_ready, "MAP_UNAVAILABLE", planner_enabled),
            (
                "costmap",
                costmap_status.valid,
                costmap_status.code or "COSTMAP_UNAVAILABLE",
                geometry_enabled or planner_enabled,
            ),
            ("tf", self._tf.ready, "TF_UNAVAILABLE", geometry_enabled or planner_enabled),
            (
                "keepout_filter", keepout_ready,
                "KEEPOUT_FILTER_UNAVAILABLE", planner_enabled,
            ),
            (
                "compute_path_to_pose",
                self._planner.single_ready,
                "PLANNER_UNAVAILABLE",
                planner_enabled,
            ),
            (
                "compute_path_through_poses",
                self._planner.through_ready,
                "PLANNER_UNAVAILABLE",
                planner_enabled,
            ),
        ]
        message = SystemReadiness()
        message.stamp = self.get_clock().now().to_msg()
        message.ready = all(ready or not enabled for _name, ready, _code, enabled in values)
        message.checks = [
            self._readiness_check(name, ready, code, enabled)
            for name, ready, code, enabled in values
        ]
        blockers = [
            name for name, ready, _code, enabled in values if enabled and not ready
        ]
        message.message = (
            "Module3 static verification is ready"
            if message.ready
            else "Waiting for: " + ", ".join(blockers)
        )
        self._readiness_publisher.publish(message)
        if message.ready != self._readiness_state:
            self._readiness_state = message.ready
            self.get_logger().info(message.message)

    def _on_resolution(self, message: ResolutionResult) -> None:
        self._workers.submit(self._verify_and_publish, message)

    def _verify_and_publish(self, resolution: ResolutionResult) -> None:
        dependency_failure = self._wait_for_dependencies()
        result = (
            dependency_failure
            if dependency_failure is not None
            else self._pipeline.verify(resolution)
        )
        output = VerificationResult()
        output.verified = result.verified
        output.resolution = resolution
        output.request_id = resolution.request_id
        output.error_code = result.error_code
        output.message = result.message
        output.map_revision = resolution.map_revision
        output.goal_id = resolution.goal_id
        output.canonical_name = resolution.canonical_name
        output.goal_pose = resolution.goal_pose
        output.topology_context = list(resolution.topology_context)
        output.constraints = list(resolution.constraints)
        output.strategy = resolution.strategy
        output.checks = [self._check_message(value) for value in result.checks]
        output.planning_length = result.planning_length
        output.planning_time = result.planning_time
        self._publisher.publish(output)
        level = self.get_logger().info if output.verified else self.get_logger().warning
        level(
            f"Verification {output.request_id}: "
            f"{'PASS' if output.verified else output.error_code}"
        )

    def _wait_for_dependencies(self):
        geometry_enabled = bool(
            self.get_parameter("geometry_check_enabled").value
        )
        planner_enabled = bool(
            self.get_parameter("planner_check_enabled").value
        )
        if not geometry_enabled and not planner_enabled:
            return None
        deadline = time.monotonic() + self._ready_timeout
        while time.monotonic() < deadline:
            costmap_status = self._geometry.status
            geometry_ready = (
                not (geometry_enabled or planner_enabled)
                or (costmap_status.valid and self._tf.ready)
            )
            planner_ready = (
                not planner_enabled or (self._planner.ready and self._mask.ready)
            )
            if geometry_ready and planner_ready:
                return None
            time.sleep(0.05)

        costmap_status = self._geometry.status
        if (geometry_enabled or planner_enabled) and not costmap_status.valid:
            return self._pipeline.dependency_failure(
                "geometry", costmap_status.code, costmap_status.message
            )
        if (geometry_enabled or planner_enabled) and not self._tf.ready:
            return self._pipeline.dependency_failure(
                "geometry",
                "TF_UNAVAILABLE",
                "Transform odom -> base_link is unavailable",
            )
        if planner_enabled and not self._mask.ready:
            return self._pipeline.dependency_failure(
                "planner",
                "KEEPOUT_MASK_UNAVAILABLE",
                "Validation keepout mask has no static map",
            )
        return self._pipeline.dependency_failure(
            "planner",
            "PLANNER_UNAVAILABLE",
            "Validation planner actions are unavailable",
        )

    def _legacy_service(self, request, response):
        node = self._repository.graph.get_node(request.goal_id)
        response.verified = node is not None
        response.failed_checks = [] if node is not None else ["ENTITY_NOT_FOUND"]
        response.explanation = (
            "Entity exists; use the canonical topic for live four-stage verification"
            if node is not None
            else "Target entity does not exist"
        )
        response.goal_x = request.goal_x
        response.goal_y = request.goal_y
        return response

    def destroy_node(self):
        self._workers.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    @staticmethod
    def _check_message(value) -> CheckResult:
        message = CheckResult()
        message.name = value.name
        message.status = value.status
        message.code = value.code
        message.message = value.message
        return message

    @staticmethod
    def _readiness_check(
        name: str, ready: bool, code: str, enabled: bool = True
    ) -> CheckResult:
        message = CheckResult()
        message.name = name
        if not enabled:
            message.status = "SKIPPED"
            message.code = "RESEARCH_GATE_DISABLED"
            message.message = "Dependency bypassed by a startup research valve"
        else:
            message.status = "PASS" if ready else "WAITING"
            message.code = "" if ready else code
            message.message = ""
        return message

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VerificationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
