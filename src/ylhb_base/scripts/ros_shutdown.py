#!/usr/bin/env python3
"""Small ROS 2 shutdown helpers shared by the Python wrapper nodes."""

from __future__ import annotations

from typing import Any


def cleanup_ros_node(node: Any, rclpy_module: Any) -> None:
    """Release a node without turning a late SIGINT into a false crash."""
    try:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # ros2 launch may deliver SIGINT while rclpy is already unwinding.
            pass
    finally:
        try:
            rclpy_module.try_shutdown()
        except KeyboardInterrupt:
            pass
