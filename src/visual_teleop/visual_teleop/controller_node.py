#!/usr/bin/env python3
"""
Controller Node - Boilerplate

Subscribes to /target/pose (visual_teleop_msgs/TrackedTarget), computes
velocity commands using a simple proportional controller, publishes /cmd_vel
for TurtleBot3.
"""

import rclpy
from rclpy.node import Node
from visual_teleop_msgs.msg import TrackedTarget
from geometry_msgs.msg import Twist


class ControllerNode(Node):
    """Node for converting target pose to velocity commands."""

    def __init__(self):
        super().__init__('controller_node')

        # Declare parameters (loaded from config/params.yaml)
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('max_angular_speed', 2.84)
        self.declare_parameter('target_distance', 1.0)  # desired distance from target (meters)
        self.declare_parameter('deadband', 0.1)  # distance deadband (meters)

        # Get parameter values
        self.linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        self.angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.deadband = self.get_parameter('deadband').get_parameter_value().double_value

        self.get_logger().info(f'Controller node initialized')
        self.get_logger().info(f'  linear_gain: {self.linear_gain}')
        self.get_logger().info(f'  angular_gain: {self.angular_gain}')
        self.get_logger().info(f'  max_linear_speed: {self.max_linear_speed}')
        self.get_logger().info(f'  max_angular_speed: {self.max_angular_speed}')
        self.get_logger().info(f'  target_distance: {self.target_distance}')
        self.get_logger().info(f'  deadband: {self.deadband}')

        # Subscriber to target pose
        self.target_sub = self.create_subscription(
            TrackedTarget,
            '/target/pose',
            self.target_callback,
            10
        )

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Store latest target
        self.latest_target = None

    def target_callback(self, msg: TrackedTarget):
        """Callback for target pose messages."""
        self.latest_target = msg
        # TODO: Compute and publish cmd_vel based on target info

    def compute_cmd_vel(self):
        """Compute velocity command from latest target."""
        # TODO:
        # 1. Check if target is recent and target_visible is True
        # 2. Extract target position (x, y) from message
        # 3. Compute error from image center (for angular control)
        # 4. Compute distance estimate from confidence/bbox size (for linear control)
        # 5. Apply proportional control with gains
        # 6. Clamp to max speeds
        # 7. Apply deadband
        # 8. Publish Twist message
        pass

    def destroy_node(self):
        # TODO: Publish zero velocity on shutdown
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()