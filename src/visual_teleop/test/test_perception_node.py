#!/usr/bin/env python3
"""
Test for Perception Node

Uses launch_testing to test node startup and basic functionality.
"""

import unittest
import launch
import launch_ros
import launch_testing
import pytest
from launch_testing.actions import ReadyToTest


# This test is a placeholder - it will be implemented when perception_node has logic
@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for testing perception_node."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='visual_teleop',
            executable='perception_node',
            name='perception_node',
            output='screen',
        ),
        ReadyToTest(),
    ])


class TestPerceptionNode(unittest.TestCase):
    """Test cases for perception_node."""

    def test_node_starts(self, proc_output):
        """Test that the node starts without errors."""
        # Wait for node to start
        proc_output.assertWaitFor('Perception node initialized', timeout=10)

    def test_node_publishes_target_pose(self):
        """Test that node publishes on /target/pose topic."""
        # TODO: Implement when node has actual publishing logic
        # Use rclpy to subscribe and verify message format
        pass

    @classmethod
    def tearDownClass(cls):
        pass


if __name__ == '__main__':
    unittest.main()