"""Credential-free ROS 2 mocks preserving the production Scout interfaces.

The public repository deliberately keeps action and topic contracts while replacing
private perception, cloud reasoning, and arm control with deterministic behavior.
"""

from __future__ import annotations

import copy
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from groundingdino_vlm_interfaces.action import VerifyTarget
from groundingdino_vlm_interfaces.msg import DetectionCandidate, TargetVerification
from obstacle_traversal_interfaces.action import AssessPushability, ApproachObstacle, ProbePushability
from obstacle_traversal_interfaces.msg import (
    FilterAuthorization, MechanicalAssessment, ObstacleDetection,
    PushabilityAssessment, TraversalStatus,
)
from semantic_navigation_interfaces.msg import NavigationTaskStatus, SystemReadiness


def _transient() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MockVerifyTarget(Node):
    def __init__(self) -> None:
        super().__init__("groundingdino_verify_target_server")
        self.declare_parameter("action_name", "/groundingdino_vlm/verify_target")
        self.declare_parameter("ready_topic", "/groundingdino_vlm/ready")
        self._ready = self.create_publisher(Bool, str(self.get_parameter("ready_topic").value), _transient())
        self._server = ActionServer(self, VerifyTarget, str(self.get_parameter("action_name").value),
                                    execute_callback=self._execute, goal_callback=self._goal,
                                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                                    callback_group=ReentrantCallbackGroup())
        self.create_timer(1.0, self._publish_ready)
        self._publish_ready()

    @staticmethod
    def _goal(goal: VerifyTarget.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT if str(goal.request_id).strip() and str(goal.vocabulary).strip() else GoalResponse.REJECT

    def _publish_ready(self) -> None:
        self._ready.publish(Bool(data=True))

    def _execute(self, handle):
        goal = handle.request
        feedback = VerifyTarget.Feedback()
        feedback.state = VerifyTarget.Feedback.DETECTING
        feedback.message = "Local mock curtain detector"
        handle.publish_feedback(feedback)
        feedback.state = VerifyTarget.Feedback.VLM_PENDING
        feedback.message = "Local mock target verification"
        handle.publish_feedback(feedback)
        verification = TargetVerification()
        verification.header = copy.deepcopy(goal.image.header)
        verification.request_id = str(goal.request_id)
        verification.roi = copy.deepcopy(goal.roi)
        verification.target_roi = copy.deepcopy(goal.target_roi)
        if verification.target_roi.width <= 0:
            verification.target_roi.x_offset, verification.target_roi.y_offset = 160, 80
            verification.target_roi.width, verification.target_roi.height = 320, 320
        verification.target_revision = 1
        verification.vlm_request_id = 1
        verification.raw_target_label = str(goal.vocabulary)
        verification.target_label = "curtain"
        verification.state = TargetVerification.VERIFIED
        verification.dino_detected = True
        verification.vlm_accepted = True
        verification.verified = True
        verification.vlm_provider = "local-mock"
        verification.vlm_model = "curtain-fixture"
        verification.dino_latency_ms = 0.1
        verification.vlm_latency_ms = 0.1
        verification.message = "Deterministic curtain detection accepted"
        candidate = DetectionCandidate()
        candidate.phrase = "curtain"
        candidate.grounding_confidence = 0.95
        candidate.x_min, candidate.y_min, candidate.x_max, candidate.y_max = 160, 80, 480, 400
        verification.candidates.append(candidate)
        result = VerifyTarget.Result()
        result.verification = verification
        handle.succeed()
        return result


class MockAssessPushability(Node):
    def __init__(self) -> None:
        super().__init__("llmdecision_pushability_server")
        self.declare_parameter("action_name", "/llmdecision/assess_pushability")
        self.declare_parameter("ready_topic", "/llmdecision/ready")
        self._ready = self.create_publisher(Bool, str(self.get_parameter("ready_topic").value), _transient())
        self._server = ActionServer(self, AssessPushability, str(self.get_parameter("action_name").value),
                                    execute_callback=self._execute,
                                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                                    callback_group=ReentrantCallbackGroup())
        self.create_timer(1.0, self._publish_ready)
        self._publish_ready()

    def _publish_ready(self) -> None:
        self._ready.publish(Bool(data=True))

    def _execute(self, handle):
        feedback = AssessPushability.Feedback()
        feedback.state = AssessPushability.Feedback.PERCEPTION
        feedback.message = "Local mock curtain perception"
        handle.publish_feedback(feedback)
        feedback.state = AssessPushability.Feedback.DECISION
        feedback.message = "Local mock pushability decision"
        handle.publish_feedback(feedback)
        goal = handle.request
        assessment = PushabilityAssessment()
        assessment.header = copy.deepcopy(goal.image.header)
        assessment.request_id = str(goal.request_id)
        assessment.roi = copy.deepcopy(goal.roi)
        assessment.target_roi = copy.deepcopy(goal.target_roi)
        assessment.state = PushabilityAssessment.ACCEPTED
        assessment.object_category = "flexible_curtain"
        assessment.pushability_probability = 0.98
        assessment.action_names = ["push", "avoid", "stop"]
        assessment.action_probabilities = [0.98, 0.01, 0.01]
        assessment.risk = "low"
        assessment.perception_provider = "local-mock"
        assessment.decision_provider = "local-mock"
        assessment.decision_model = "curtain-fixture"
        assessment.message = "Curtain is accepted as pushable in public fixture"
        result = AssessPushability.Result()
        result.assessment = assessment
        handle.succeed()
        return result


class MockProbePushability(Node):
    def __init__(self) -> None:
        super().__init__("piper_probe_server")
        self.declare_parameter("action_name", "/piper/probe_pushability")
        self.declare_parameter("assessment_topic", "/obstacle_traversal/mechanical_assessment")
        self._assessment = self.create_publisher(MechanicalAssessment, str(self.get_parameter("assessment_topic").value), _transient())
        self._server = ActionServer(self, ProbePushability, str(self.get_parameter("action_name").value),
                                    execute_callback=self._execute,
                                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                                    callback_group=ReentrantCallbackGroup())

    def _execute(self, handle):
        goal = handle.request
        for phase, message in ((ProbePushability.Feedback.POSITIONING, "Static arm pose"),
                               (ProbePushability.Feedback.SWEEPING, "No-op public arm sweep"),
                               (ProbePushability.Feedback.RETURNING, "Static arm retraction")):
            feedback = ProbePushability.Feedback()
            feedback.phase, feedback.progress, feedback.resistance_ratio = phase, 1.0, 0.1
            feedback.message = message
            handle.publish_feedback(feedback)
        assessment = MechanicalAssessment()
        assessment.request_id = str(goal.request_id)
        assessment.state = MechanicalAssessment.FLEXIBLE
        assessment.contact_probability = 0.95
        assessment.mobility_probability = 0.98
        assessment.mechanical_probability = 0.97
        assessment.resistance_ratio = 0.1
        assessment.vision_classification = MechanicalAssessment.VISION_CURTAIN
        assessment.vision_confidence = 0.95
        assessment.vision_backend_ok = True
        assessment.vision_votes, assessment.vision_samples = 3, 3
        assessment.vision_reason = "local mock curtain fixture"
        assessment.arm_returned = True
        assessment.message = "Mock probe completed without actuator motion"
        self._assessment.publish(assessment)
        result = ProbePushability.Result()
        result.assessment = assessment
        handle.succeed()
        return result


class MockApproachObstacle(Node):
    """Keep the production approach action available without commanding motion."""

    def __init__(self) -> None:
        super().__init__("mock_obstacle_approach_controller")
        self.declare_parameter("action_name", "/obstacle_traversal/approach_obstacle")
        self._server = ActionServer(
            self,
            ApproachObstacle,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

    @staticmethod
    def _goal(goal: ApproachObstacle.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT if str(goal.request_id).strip() else GoalResponse.REJECT

    def _execute(self, handle):
        goal = handle.request
        feedback = ApproachObstacle.Feedback()
        feedback.phase = ApproachObstacle.Feedback.APPROACHING
        feedback.target_clearance_m = float(goal.target_clearance_m)
        feedback.global_clearance_m = float(goal.target_clearance_m)
        feedback.cross_track_error_m = 0.0
        feedback.heading_error_rad = 0.0
        handle.publish_feedback(feedback)

        result = ApproachObstacle.Result()
        result.reached = True
        result.stopped = True
        result.final_clearance_m = max(float(goal.target_clearance_m), 0.20)
        result.traveled_distance_m = 0.0
        result.error_code = ""
        result.message = "Public mock approach completed without velocity command"
        handle.succeed()
        return result


class MockYolo(Node):
    def __init__(self) -> None:
        super().__init__("piper_yolo_obstacle_detector")
        self.declare_parameter("result_topic", "/piper/yolo_obstacle_result")
        self.declare_parameter("ready_topic", "/piper/yolo_ready")
        self._result = self.create_publisher(ObstacleDetection, str(self.get_parameter("result_topic").value), 10)
        self._ready = self.create_publisher(Bool, str(self.get_parameter("ready_topic").value), _transient())
        self.create_timer(0.5, self._publish)
        self._publish()

    def _publish(self) -> None:
        self._ready.publish(Bool(data=True))
        detection = ObstacleDetection()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = "map"
        detection.source = "local-mock"
        detection.classification = ObstacleDetection.CURTAIN
        detection.confidence = 0.95
        detection.backend_ok = True
        detection.reason = "deterministic living-room curtain fixture"
        self._result.publish(detection)


class MockPiperArm(Node):
    def __init__(self) -> None:
        super().__init__("mock_piper_arm")
        self.declare_parameter("joint_state_topic", "/isaac_joint_states")
        self.declare_parameter("joint_command_topic", "/isaac_joint_command")
        self._state = self.create_publisher(JointState, str(self.get_parameter("joint_state_topic").value), 10)
        self.create_subscription(JointState, str(self.get_parameter("joint_command_topic").value), self._on_command, 10)
        self._positions = [0.0, 0.8, -1.0, 0.0, 0.287, 0.0, 0.02, -0.02]
        self._last_command = None
        self.create_timer(0.1, self._publish)

    def _on_command(self, message: JointState) -> None:
        self._last_command = message

    def _publish(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f"joint{i}" for i in range(1, 9)]
        message.position = list(self._positions)
        message.velocity = [0.0] * 8
        self._state.publish(message)


class FixedRouteDriver(Node):
    """Publish the verified curtain route and a terminal navigation status."""
    def __init__(self) -> None:
        super().__init__("fixed_route_driver")
        self.declare_parameter("goal_x", 4.20)
        self.declare_parameter("goal_y", -0.004)
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("obstacle_x", 3.29)
        self.declare_parameter("obstacle_y", -0.004)
        self.declare_parameter("arrival_distance_m", 0.30)
        self.declare_parameter("arrival_hold_s", 0.75)
        self.declare_parameter("timeout_s", 300.0)
        self.declare_parameter("require_authorization", False)
        self._goal = (float(self.get_parameter("goal_x").value), float(self.get_parameter("goal_y").value))
        self._obstacle = (
            float(self.get_parameter("obstacle_x").value),
            float(self.get_parameter("obstacle_y").value),
        )
        self._odom = None
        self._odom_received = False
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._authorized = not bool(self.get_parameter("require_authorization").value)
        self._published = False
        self._terminal = False
        self._arrival_started = None
        self._started = time.monotonic()
        transient = _transient()
        self._clicked = self.create_publisher(PointStamped, "/clicked_point", 10)
        self._nav = self.create_publisher(NavigationTaskStatus, "/semantic_navigation/status", transient)
        self._traversal = self.create_publisher(TraversalStatus, "/obstacle_traversal/status", transient)
        self._traversal_ready = self.create_publisher(Bool, "/obstacle_traversal/ready", transient)
        self._semantic_ready = self.create_publisher(SystemReadiness, "/semantic_navigation/readiness", transient)
        self._verification_ready = self.create_publisher(SystemReadiness, "/semantic_navigation/verification_readiness", transient)
        self._filter_authorization = self.create_publisher(
            FilterAuthorization, "/obstacle_traversal/filter_authorization", transient
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(TraversalStatus, "/obstacle_traversal/status", self._on_authorization, transient)
        self.create_timer(0.1, self._tick)

    def _on_odom(self, message: Odometry) -> None:
        self._odom_received = True

    def _map_robot_xy(self):
        """Return the robot pose in the same local map frame as the route."""
        try:
            transform = self._tf.lookup_transform(
                str(self.get_parameter("goal_frame").value),
                "base_link",
                rclpy.time.Time(),
            ).transform
        except TransformException:
            return None
        return (
            float(transform.translation.x),
            float(transform.translation.y),
        )

    def _locked_path(self, start):
        """Build a deterministic, request-locked path for the velocity gate."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = str(self.get_parameter("goal_frame").value)
        steps = 40
        dx = self._goal[0] - start[0]
        dy = self._goal[1] - start[1]
        heading = math.atan2(dy, dx)
        for index in range(steps + 1):
            fraction = index / float(steps)
            pose = PoseStamped()
            pose.header = copy.deepcopy(path.header)
            pose.pose.position.x = start[0] + fraction * dx
            pose.pose.position.y = start[1] + fraction * dy
            pose.pose.orientation.z = math.sin(heading / 2.0)
            pose.pose.orientation.w = math.cos(heading / 2.0)
            path.poses.append(pose)
        return path

    def _on_authorization(self, message: TraversalStatus) -> None:
        self._authorized = self._authorized or int(message.state) in (TraversalStatus.AUTHORIZED, TraversalStatus.TRAVERSING)

    def _publish(self, state: str, terminal: bool = False, message: str = "", failure: str = "") -> None:
        status = NavigationTaskStatus()
        status.task_id = "public-curtain-route"
        status.request_id = "public-curtain-route"
        status.state, status.terminal = state, terminal
        status.failure_code, status.message = failure, message
        status.distance_remaining = float(math.dist(self._odom, self._goal)) if self._odom else float("inf")
        status.planning_attempt = 1
        self._nav.publish(status)

    def _publish_traversal(self, state: int, message: str) -> None:
        status = TraversalStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.task_id = "public-curtain-route"
        status.request_id = "public-curtain-route"
        status.obstacle_id = "curtain"
        status.state, status.message = state, message
        status.navigation_paused = state in (TraversalStatus.AUTHORIZED, TraversalStatus.TRAVERSING)
        status.filter_authorized = self._authorized
        status.dino_verified = True
        status.pushability_probability = 0.98
        status.dino_probability = 0.95
        status.mechanical_probability = 0.97
        status.fusion_probability = 0.97
        status.arm_returned = True
        self._traversal.publish(status)

    def _publish_readiness(self) -> None:
        self._traversal_ready.publish(Bool(data=True))
        for publisher in (self._semantic_ready, self._verification_ready):
            readiness = SystemReadiness()
            readiness.stamp = self.get_clock().now().to_msg()
            readiness.ready = True
            readiness.message = "Public deterministic route provider is ready"
            publisher.publish(readiness)

    def _tick(self) -> None:
        if self._terminal:
            return
        if time.monotonic() - self._started > float(self.get_parameter("timeout_s").value):
            self._publish("FAILED", True, "Waiting for /odom, authorization, or route arrival timed out", "PUBLIC_ROUTE_TIMEOUT")
            self._terminal = True
            return
        if not self._odom_received:
            self._publish_readiness()
            self._publish("WAITING_FOR_ODOM", message="Waiting for Isaac /odom and map->odom->base_link TF")
            return
        self._odom = self._map_robot_xy()
        if self._odom is None:
            self._publish_readiness()
            self._publish("WAITING_FOR_TF", message="Waiting for map->odom->base_link TF")
            return
        if not self._authorized:
            self._publish("WAITING_FOR_AUTHORIZATION", message="Waiting for traversal manager authorization")
            return
        self._publish_readiness()
        if not self._published and self._clicked.get_subscription_count() > 0:
            goal = PointStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = str(self.get_parameter("goal_frame").value)
            goal.point.x, goal.point.y = self._goal
            self._clicked.publish(goal)
            self._published = True
            authorization = FilterAuthorization()
            authorization.header = copy.deepcopy(goal.header)
            authorization.request_id = "public-curtain-route"
            authorization.authorized = True
            authorization.obstacle_center_map.x = self._obstacle[0]
            authorization.obstacle_center_map.y = self._obstacle[1]
            # Keep the authorization valid through Isaac startup and the full
            # 4.2 m route; the scan filter still enforces its own bounded
            # timeout and revokes on pass completion.
            authorization.maximum_duration_s = 120.0
            authorization.pass_distance_m = 0.80
            authorization.locked_path = self._locked_path(self._odom)
            self._filter_authorization.publish(authorization)
            self._publish_traversal(TraversalStatus.AUTHORIZED, "Public mock authorization granted")
        self._publish("NAVIGATING", message="Following fixed living-room curtain route")
        self._publish_traversal(TraversalStatus.TRAVERSING, "Following local NeuPAN route")
        distance = math.dist(self._odom, self._goal)
        if distance <= float(self.get_parameter("arrival_distance_m").value):
            self._arrival_started = self._arrival_started or time.monotonic()
            if time.monotonic() - self._arrival_started >= float(self.get_parameter("arrival_hold_s").value):
                self._publish("SUCCEEDED", True, "Scout Mini crossed the curtain route")
                self._publish_traversal(TraversalStatus.FUSING, "Curtain traversal completed")
                self._terminal = True
        else:
            self._arrival_started = None


def _spin(node_factory) -> None:
    rclpy.init()
    node = node_factory()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def verify_target_main() -> None: _spin(MockVerifyTarget)
def assess_pushability_main() -> None: _spin(MockAssessPushability)
def probe_pushability_main() -> None: _spin(MockProbePushability)
def approach_obstacle_main() -> None: _spin(MockApproachObstacle)
def yolo_main() -> None: _spin(MockYolo)
def arm_main() -> None: _spin(MockPiperArm)
def route_main() -> None: _spin(FixedRouteDriver)
