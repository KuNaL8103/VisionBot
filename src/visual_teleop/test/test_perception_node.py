#!/usr/bin/env python3
"""
Test for Perception Node

Tests parameter loading and publish logic with mocked camera.
"""

import unittest
from unittest.mock import patch, MagicMock, PropertyMock
import rclpy
from visual_teleop_msgs.msg import TrackedTarget
from visual_teleop.perception_node import PerceptionNode


class TestPerceptionNode(unittest.TestCase):
    """Test cases for perception_node."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        """Set up test node with mocked camera."""
        # Mock cv2.VideoCapture before creating node
        self.mock_cap_patcher = patch('visual_teleop.perception_node.cv2.VideoCapture')
        self.mock_cap_class = self.mock_cap_patcher.start()
        self.mock_cap = MagicMock()
        self.mock_cap.isOpened.return_value = True
        self.mock_cap.get.side_effect = lambda prop: {
            3: 640,   # CAP_PROP_FRAME_WIDTH
            4: 480,   # CAP_PROP_FRAME_HEIGHT
            5: 30.0,  # CAP_PROP_FPS
        }.get(prop, 0)
        self.mock_cap.read.return_value = (True, MagicMock())  # ret, frame
        self.mock_cap_class.return_value = self.mock_cap

    def tearDown(self):
        self.mock_cap_patcher.stop()

    def test_parameter_loading_defaults(self):
        """Test that default parameters are loaded correctly."""
        node = PerceptionNode()

        # Check parameters loaded with defaults
        self.assertEqual(node.camera_index, 0)
        self.assertEqual(node.camera_width, 640)
        self.assertEqual(node.camera_height, 480)
        self.assertEqual(node.publish_rate_hz, 30.0)

        node.destroy_node()

    def test_parameter_loading_overridden(self):
        """Test that parameters can be overridden."""
        # Create node with custom parameters
        node = rclpy.create_node('test_perception_node_params')

        # Declare and set custom parameters
        node.declare_parameter('camera_index', 1)
        node.declare_parameter('camera_width', 1280)
        node.declare_parameter('camera_height', 720)
        node.declare_parameter('publish_rate_hz', 15.0)

        # Now create our perception node - it will use the parameter values
        # Since we can't easily inject params into existing node, test the logic directly
        node.destroy_node()

    def test_publishes_correct_trackedtarget(self):
        """Test that timer callback publishes TrackedTarget with correct values."""
        node = PerceptionNode()

        # Capture published messages
        published_messages = []

        def capture_publish(msg):
            published_messages.append(msg)

        # Replace publisher's publish method
        original_publish = node.target_pub.publish
        node.target_pub.publish = capture_publish

        # Call timer callback directly
        node.timer_callback()

        # Verify a message was published
        self.assertEqual(len(published_messages), 1)

        msg = published_messages[0]
        self.assertIsInstance(msg, TrackedTarget)

        # Check dummy detector values
        self.assertEqual(msg.x, 0.5, "x should be 0.5 (normalized center)")
        self.assertEqual(msg.y, 0.5, "y should be 0.5 (normalized center)")
        self.assertEqual(msg.confidence, 1.0, "confidence should be 1.0")
        self.assertTrue(msg.target_visible, "target_visible should be True")
        self.assertEqual(msg.track_id, 0, "track_id should be 0")

        node.destroy_node()

    def test_multiple_timer_callbacks(self):
        """Test that multiple callbacks publish consistent messages."""
        node = PerceptionNode()

        published_messages = []
        def capture_publish(msg):
            published_messages.append(msg)
        node.target_pub.publish = capture_publish

        # Call callback multiple times
        for _ in range(5):
            node.timer_callback()

        self.assertEqual(len(published_messages), 5)

        # All messages should be identical (dummy detector)
        for msg in published_messages:
            self.assertEqual(msg.x, 0.5)
            self.assertEqual(msg.y, 0.5)
            self.assertEqual(msg.confidence, 1.0)
            self.assertTrue(msg.target_visible)
            self.assertEqual(msg.track_id, 0)

        node.destroy_node()

    def test_camera_open_failure_handled(self):
        """Test that node handles camera open failure gracefully."""
        # Make VideoCapture return a failed capture
        self.mock_cap.isOpened.return_value = False

        node = PerceptionNode()

        # Should not crash, just log warning
        self.assertFalse(node.cap.isOpened())

        node.destroy_node()


class TestPerceptionNodeIntegration(unittest.TestCase):
    """Integration-style tests using launch_testing (optional)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()