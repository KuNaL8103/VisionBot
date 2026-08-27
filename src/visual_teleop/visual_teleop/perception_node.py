#!/usr/bin/env python3
"""
Perception Node - Boilerplate

Subscribes to camera image topic, runs YOLO + ByteTrack to detect/track target,
publishes target pose on /target/pose.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped


class PerceptionNode(Node):
    """Node for detecting and tracking a target from webcam feed."""

    def __init__(self):
        super().__init__('perception_node')

        # Declare parameters (loaded from config/params.yaml)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('tracking_max_age', 30)

        # Get parameter values
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.tracking_max_age = self.get_parameter('tracking_max_age').get_parameter_value().integer_value

        self.get_logger().info(f'Perception node initialized')
        self.get_logger().info(f'  camera_index: {self.camera_index}')
        self.get_logger().info(f'  target_class: {self.target_class}')
        self.get_logger().info(f'  confidence_threshold: {self.confidence_threshold}')
        self.get_logger().info(f'  tracking_max_age: {self.tracking_max_age}')

        # TODO: Initialize YOLO model, ByteTrack tracker, OpenCV VideoCapture
        # TODO: Create publisher for /target/pose (PoseStamped)
        # TODO: Create timer callback for processing frames

        # Placeholder: publisher for target pose
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target/pose', 10)

        # Placeholder: timer for processing loop
        # self.timer = self.create_timer(1.0/30.0, self.process_frame)

    def process_frame(self):
        """Process a single frame from the camera."""
        # TODO:
        # 1. Read frame from camera
        # 2. Run YOLO detection
        # 3. Update ByteTrack tracker
        # 4. Filter for target class
        # 5. Select best track (e.g., highest confidence, closest to center)
        # 6. Convert track to PoseStamped (x, y in image coords -> 3D pose estimate)
        # 7. Publish on /target/pose
        pass

    def destroy_node(self):
        # TODO: Release camera, cleanup resources
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()