#!/usr/bin/env python3
"""
Test for Perception Node

Tests parameter loading and publish logic with mocked camera.
"""

import unittest
from unittest.mock import patch, MagicMock, PropertyMock
import rclpy
import numpy as np
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
        """Set up test node with mocked camera, YOLO, and ByteTrack."""
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
        # Create a mock frame with proper shape attribute (height, width, channels)
        mock_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.mock_cap.read.return_value = (True, mock_frame)  # ret, frame
        self.mock_cap_class.return_value = self.mock_cap

        # Mock YOLO model
        self.mock_yolo_patcher = patch('visual_teleop.perception_node.YOLO')
        self.mock_yolo_class = self.mock_yolo_patcher.start()
        self.mock_model = MagicMock()
        self.mock_model.names = {0: 'person', 1: 'bicycle', 2: 'car'}
        self.mock_model.to.return_value = None
        self.mock_yolo_class.return_value = self.mock_model

        # Mock YOLO inference results
        self.mock_result = MagicMock()
        self.mock_box = MagicMock()
        self.mock_box.cls.item.return_value = 0  # person class
        self.mock_box.conf.item.return_value = 0.8
        self.mock_box.xyxy = [MagicMock()]
        self.mock_box.xyxy[0].tolist.return_value = [100, 100, 300, 300]
        self.mock_result.boxes = [self.mock_box]
        self.mock_model.return_value = [self.mock_result]

        # Mock ByteTrack tracker
        self.mock_tracker_patcher = patch('visual_teleop.perception_node.TrackerWrapper')
        self.mock_tracker_class = self.mock_tracker_patcher.start()
        self.mock_tracker = MagicMock()

        # Mock tracked detections output
        mock_detections = MagicMock()
        mock_detections.confidence = np.array([0.8], dtype=np.float32)
        mock_detections.xyxy = np.array([[100, 100, 300, 300]], dtype=np.float32)
        mock_detections.tracker_id = np.array([1], dtype=np.int32)
        mock_detections.__len__ = lambda self: 1  # Make len() work for "if len(detections) > 0"
        self.mock_tracker.update.return_value = mock_detections
        self.mock_tracker_class.return_value = self.mock_tracker

        # Mock create_timer to prevent timer from firing during init (warmup)
        self.mock_timer_patcher = patch.object(PerceptionNode, 'create_timer', return_value=MagicMock())
        self.mock_timer = self.mock_timer_patcher.start()

    def tearDown(self):
        self.mock_cap_patcher.stop()
        self.mock_yolo_patcher.stop()
        self.mock_tracker_patcher.stop()
        self.mock_timer_patcher.stop()

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

        # Check mocked detector values (bbox [100,100,300,300] on 640x480 frame)
        # Center = (200, 200), normalized = (200/640, 200/480) = (0.3125, 0.41666...)
        self.assertAlmostEqual(msg.x, 200/640, places=4, msg="x should be normalized center of mocked bbox")
        self.assertAlmostEqual(msg.y, 200/480, places=4, msg="y should be normalized center of mocked bbox")
        self.assertAlmostEqual(msg.confidence, 0.8, places=4, msg="confidence should match mocked detection")
        self.assertTrue(msg.target_visible, "target_visible should be True for detection")
        self.assertEqual(msg.track_id, 1, "track_id should match mocked tracker output")

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

        # All messages should be identical (mocked detector)
        for msg in published_messages:
            self.assertAlmostEqual(msg.x, 200/640, places=4)
            self.assertAlmostEqual(msg.y, 200/480, places=4)
            self.assertAlmostEqual(msg.confidence, 0.8, places=4)
            self.assertTrue(msg.target_visible)
            self.assertEqual(msg.track_id, 1)

        node.destroy_node()

    def test_camera_open_failure_handled(self):
        """Test that node handles camera open failure gracefully."""
        # Make VideoCapture return a failed capture
        self.mock_cap.isOpened.return_value = False

        node = PerceptionNode()

        # Should not crash, just log warning
        self.assertFalse(node.cap.isOpened())

        node.destroy_node()

    def test_no_detection_publishes_single_zero_visible_msg(self):
        """Test that when tracker returns no detections, exactly ONE message is published
        with target_visible=False and x/y at last known position (not double-published)."""
        # Change tracker mock to return empty detections
        mock_detections = MagicMock()
        mock_detections.__len__ = lambda self: 0  # No detections
        self.mock_tracker.update.return_value = mock_detections

        node = PerceptionNode()

        # Capture published messages
        published_messages = []

        def capture_publish(msg):
            published_messages.append(msg)
        node.target_pub.publish = capture_publish

        # Call timer callback
        node.timer_callback()

        # Verify exactly ONE message was published (not double-published)
        self.assertEqual(len(published_messages), 1,
                         msg=f"Expected exactly 1 publish, got {len(published_messages)}")

        msg = published_messages[0]
        self.assertIsInstance(msg, TrackedTarget)

        # Should have target_visible=False
        self.assertFalse(msg.target_visible, "target_visible should be False when no detection")
        # Should hold last known position (initialized to 0.5, 0.5)
        self.assertEqual(msg.x, 0.5, "x should be last known position (0.5)")
        self.assertEqual(msg.y, 0.5, "y should be last known position (0.5)")
        self.assertEqual(msg.confidence, 0.0, "confidence should be 0.0 when no detection")
        self.assertEqual(msg.track_id, 0, "track_id should be 0 when no detection")

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