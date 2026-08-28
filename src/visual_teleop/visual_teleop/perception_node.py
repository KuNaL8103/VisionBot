#!/usr/bin/env python3
"""
Perception Node - Dummy Detector

Opens webcam using OpenCV VideoCapture, reads frames at configured rate,
publishes a fixed-center TrackedTarget message on /target/pose.
This is a placeholder - no ML detection yet.
"""

import rclpy
from rclpy.node import Node
from visual_teleop_msgs.msg import TrackedTarget
import cv2


class PerceptionNode(Node):
    """Node for publishing dummy target at frame center."""

    def __init__(self):
        super().__init__('perception_node')

        # Declare parameters (loaded from config/params.yaml)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('publish_rate_hz', 30.0)

        # Get parameter values
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.camera_width = self.get_parameter('camera_width').get_parameter_value().integer_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.get_logger().info(f'Perception node initialized')
        self.get_logger().info(f'  camera_index: {self.camera_index}')
        self.get_logger().info(f'  camera_width: {self.camera_width}')
        self.get_logger().info(f'  camera_height: {self.camera_height}')
        self.get_logger().info(f'  publish_rate_hz: {self.publish_rate_hz}')

        # Open webcam
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().warn(f'Could not open camera at index {self.camera_index}')
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f'  Camera opened: {int(actual_width)}x{int(actual_height)} @ {actual_fps:.1f} FPS')

        # Publisher for target info
        self.target_pub = self.create_publisher(TrackedTarget, '/target/pose', 10)

        # Timer for processing loop
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

    def timer_callback(self):
        """Timer callback: read frame and publish dummy TrackedTarget."""
        # Read frame (we don't use it for detection yet, but we read to keep camera active)
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera', throttle_duration_sec=5.0)

        # Create and publish dummy TrackedTarget message
        # Fixed center: x=0.5, y=0.5 (normalized), confidence=1.0, visible=true, track_id=0
        msg = TrackedTarget()
        msg.x = 0.5
        msg.y = 0.5
        msg.confidence = 1.0
        msg.target_visible = True
        msg.track_id = 0
        self.target_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
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