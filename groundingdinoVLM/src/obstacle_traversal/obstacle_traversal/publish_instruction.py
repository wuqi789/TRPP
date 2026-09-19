#!/usr/bin/env python3
"""Publish one arbitrary natural-language navigation instruction safely."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--min-subscriptions", type=int, default=1,
        help="number of matched subscribers required before publishing",
    )
    parser.add_argument(
        "--delivery-wait", type=float, default=0.5,
        help="seconds to keep the reliable publisher alive after publishing",
    )
    parser.add_argument("instruction", nargs="+", help="natural-language instruction")
    parsed = parser.parse_args(args)
    instruction = " ".join(parsed.instruction).strip()
    if not instruction:
        parser.error("instruction must not be empty")
    if parsed.min_subscriptions < 1:
        parser.error("--min-subscriptions must be at least 1")
    if parsed.delivery_wait < 0.0:
        parser.error("--delivery-wait must not be negative")

    rclpy.init()
    node = rclpy.create_node("navigation_instruction_publisher")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(String, "/user_instruction", qos)
    deadline = time.monotonic() + 10.0
    while (
        publisher.get_subscription_count() < parsed.min_subscriptions
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.1)
    matched = publisher.get_subscription_count()
    if matched < parsed.min_subscriptions:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(
            "/user_instruction matched "
            f"{matched}/{parsed.min_subscriptions} required subscribers after 10 seconds"
        )
    publisher.publish(String(data=instruction))
    end = time.monotonic() + parsed.delivery_wait
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.get_logger().info(
        f"Navigation instruction published to {matched} matched subscribers"
    )
    node.destroy_node()
    rclpy.shutdown()
