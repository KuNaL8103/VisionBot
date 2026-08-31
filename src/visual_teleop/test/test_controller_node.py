#!/usr/bin/env python3
"""
Test for Controller Node

Uses unittest with mocked TrackedTarget inputs to test controller logic
without requiring a camera or simulation.
"""

import unittest
import rclpy
from rclpy.node import Node
from visual_teleop_msgs.msg import TrackedTarget
from geometry_msgs.msg import Twist, TwistStamped

from visual_teleop.controller_node import ControllerNode


class TestControllerNode(unittest.TestCase):
    """Test cases for controller_node logic."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context."""
        rclpy.shutdown()

    def setUp(self):
        """Create a fresh controller node for each test."""
        self.node = ControllerNode()

    def tearDown(self):
        """Clean up node."""
        self.node.destroy_node()

    def _create_target(self, x: float, y: float, target_visible: bool = True,
                       confidence: float = 0.8, track_id: int = 1) -> TrackedTarget:
        """Helper to create a TrackedTarget message."""
        msg = TrackedTarget()
        msg.x = x
        msg.y = y
        msg.confidence = confidence
        msg.target_visible = target_visible
        msg.track_id = track_id
        return msg

    def test_dead_center_produces_near_zero_angular(self):
        """Target at image center (x=0.5) should produce near-zero angular velocity."""
        target = self._create_target(x=0.5, y=0.5)
        self.node.target_callback(target)
        twist_stamped = self.node.compute_cmd_vel()

        # Angular velocity should be zero (within dead_zone_px)
        self.assertAlmostEqual(twist_stamped.twist.angular.z, 0.0, places=5,
                               msg=f"Expected angular.z ~ 0.0, got {twist_stamped.twist.angular.z}")

        # Linear velocity should be positive (target is centered)
        self.assertGreater(twist_stamped.twist.linear.x, 0.0,
                           msg=f"Expected positive linear.x, got {twist_stamped.twist.linear.x}")
        self.assertLessEqual(twist_stamped.twist.linear.x, self.node.max_linear_speed)

    def test_far_left_produces_positive_angular(self):
        """Target far left (x=0.1) should produce positive angular velocity (turn LEFT/CCW)."""
        target = self._create_target(x=0.1, y=0.5)
        self.node.target_callback(target)
        twist_stamped = self.node.compute_cmd_vel()

        # Angular velocity should be positive (turn LEFT/CCW to center target on left)
        # ROS convention: angular.z > 0 = CCW = turn LEFT
        # Target on LEFT side of image (x < 0.5) -> robot must turn LEFT -> angular.z > 0
        self.assertGreater(twist_stamped.twist.angular.z, 0.0,
                           msg=f"Expected positive angular.z for left target, got {twist_stamped.twist.angular.z}")
        # Magnitude should be clamped to max_angular_speed
        self.assertLessEqual(twist_stamped.twist.angular.z, self.node.max_angular_speed)

        # Linear velocity should be zero (target too far left, not forward-facing)
        self.assertEqual(twist_stamped.twist.linear.x, 0.0,
                         msg=f"Expected zero linear.x for far-left target, got {twist_stamped.twist.linear.x}")

    def test_far_right_produces_negative_angular(self):
        """Target far right (x=0.9) should produce negative angular velocity (turn RIGHT/CW)."""
        target = self._create_target(x=0.9, y=0.5)
        self.node.target_callback(target)
        twist_stamped = self.node.compute_cmd_vel()

        # Angular velocity should be negative (turn RIGHT/CW to center target on right)
        # ROS convention: angular.z < 0 = CW = turn RIGHT
        # Target on RIGHT side of image (x > 0.5) -> robot must turn RIGHT -> angular.z < 0
        self.assertLess(twist_stamped.twist.angular.z, 0.0,
                        msg=f"Expected negative angular.z for right target, got {twist_stamped.twist.angular.z}")
        # Magnitude should be clamped to max_angular_speed
        self.assertGreaterEqual(twist_stamped.twist.angular.z, -self.node.max_angular_speed)

        # Linear velocity should be zero (target too far right, not forward-facing)
        self.assertEqual(twist_stamped.twist.linear.x, 0.0,
                         msg=f"Expected zero linear.x for far-right target, got {twist_stamped.twist.linear.x}")

    def test_target_not_visible_produces_zero_twist(self):
        """target_visible=False should produce zero Twist (stop)."""
        target = self._create_target(x=0.5, y=0.5, target_visible=False)
        self.node.target_callback(target)
        twist_stamped = self.node.compute_cmd_vel()

        # Both linear and angular should be zero
        self.assertEqual(twist_stamped.twist.linear.x, 0.0,
                         msg=f"Expected zero linear.x when target not visible, got {twist_stamped.twist.linear.x}")
        self.assertEqual(twist_stamped.twist.angular.z, 0.0,
                         msg=f"Expected zero angular.z when target not visible, got {twist_stamped.twist.angular.z}")

    def test_no_target_received_produces_zero_twist(self):
        """No target received yet should produce zero Twist."""
        # Don't call target_callback at all
        twist_stamped = self.node.compute_cmd_vel()

        self.assertEqual(twist_stamped.twist.linear.x, 0.0,
                         msg=f"Expected zero linear.x with no target, got {twist_stamped.twist.linear.x}")
        self.assertEqual(twist_stamped.twist.angular.z, 0.0,
                         msg=f"Expected zero angular.z with no target, got {twist_stamped.twist.angular.z}")

    def test_target_within_dead_zone_produces_zero_angular(self):
        """Target within dead_zone_px of center should produce zero angular velocity."""
        # dead_zone_px default is 0.05, so target at x=0.52 is within dead zone
        target = self._create_target(x=0.52, y=0.5)
        self.node.target_callback(target)
        twist_stamped = self.node.compute_cmd_vel()

        self.assertAlmostEqual(twist_stamped.twist.angular.z, 0.0, places=5,
                               msg=f"Expected zero angular.z within dead zone, got {twist_stamped.twist.angular.z}")

    def test_angular_velocity_clamped_to_max(self):
        """Angular velocity should be clamped to max_angular_speed."""
        # Use very large error to exceed max
        target = self._create_target(x=0.0, y=0.5)  # max error = -0.5
        self.node.target_callback(target)
        twist_stamped = self.node.compute_cmd_vel()

        # With angular_gain=1.0, error=-0.5 -> angular_vel=0.5 (within max 1.82)
        # But with larger gain it would clamp. Let's verify it doesn't exceed max.
        self.assertLessEqual(abs(twist_stamped.twist.angular.z), self.node.max_angular_speed + 1e-6,
                             msg=f"Angular velocity {twist_stamped.twist.angular.z} exceeds max {self.node.max_angular_speed}")

    def test_linear_velocity_clamped_to_max(self):
        """Linear velocity should be clamped to max_linear_speed."""
        # Target centered with high confidence
        target = self._create_target(x=0.5, y=0.5, confidence=1.0)
        self.node.target_callback(target)
        twist_stamped = self.node.compute_cmd_vel()

        self.assertLessEqual(twist_stamped.twist.linear.x, self.node.max_linear_speed + 1e-6,
                             msg=f"Linear velocity {twist_stamped.twist.linear.x} exceeds max {self.node.max_linear_speed}")
        self.assertGreaterEqual(twist_stamped.twist.linear.x, 0.0)

    def test_target_slightly_left_still_moves_forward(self):
        """Target slightly left (within forward-facing range) should still move forward."""
        # forward-facing range is abs(error_x) < 0.3, so x=0.3 is at boundary
        target = self._create_target(x=0.3, y=0.5)
        self.node.target_callback(target)
        twist_stamped = self.node.compute_cmd_vel()

        # Should have positive linear velocity (moving forward)
        self.assertGreater(twist_stamped.twist.linear.x, 0.0)
        # Should have positive angular velocity (turning LEFT/CCW toward target on left)
        self.assertGreater(twist_stamped.twist.angular.z, 0.0)

    def test_target_slightly_right_still_moves_forward(self):
        """Target slightly right (within forward-facing range) should still move forward."""
        target = self._create_target(x=0.7, y=0.5)
        self.node.target_callback(target)
        twist_stamped = self.node.compute_cmd_vel()

        # Should have positive linear velocity (moving forward)
        self.assertGreater(twist_stamped.twist.linear.x, 0.0)
        # Should have negative angular velocity (turning RIGHT/CW toward target on right)
        self.assertLess(twist_stamped.twist.angular.z, 0.0)


if __name__ == '__main__':
    unittest.main()