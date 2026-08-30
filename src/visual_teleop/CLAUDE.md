# Project: Visual Teleoperation (Webcam -> Simulated Robot)

## What this is
A ROS 2 Jazzy package that uses a laptop webcam as the only sensor.
YOLO + ByteTrack track a target (hand/object/face) in the video feed.
Target position is published on /target/pose. A controller node converts
this into /cmd_vel to drive a simulated TurtleBot3 in Gazebo.

## Environment
- ROS 2 Jazzy, Ubuntu 24.04, WSL2 on Windows 11
- Python 3.12, rclpy
- Perception deps: ultralytics (YOLO), OpenCV, supervision (ByteTrack)
- Sim: TurtleBot3 Gazebo packages (turtlebot3_gazebo)
- Webcam accessed via usbipd-win USB passthrough to WSL2; opened with OpenCV
  V4L2 backend using MJPG format (YUYV produces corrupted frames; GStreamer
  backend not used)

## Package layout
See docs/ARCHITECTURE.md for the node graph and docs/TOPICS.md for message
contracts. Do not change topic names/types without updating both.

## Conventions
- All nodes are Python (rclpy), not C++.
- Params live in config/params.yaml, loaded via declare_parameter — no
  hardcoded camera index, target class, or control gains in node code.
- Every new node needs a corresponding test in test/ using launch_testing
  or a plain unittest with mocked publishers/subscribers.
- Commit messages: conventional commits (feat:, fix:, docs:, test:).

## Current status
- **visual_teleop_msgs/TrackedTarget**: exists, builds, and is visible via `ros2 interface show`
- **perception_node**: opens real webcam via cv2.VideoCapture (V4L2 backend, MJPG format), runs Ultralytics YOLOv8 (yolov8n.pt) to detect `target_class` (default "person"), uses **ByteTrack (via supervision)** for stable track IDs across frames. Publishes TrackedTarget with normalized bbox center, confidence, `target_visible`, and `track_id`. **ByteTrack CONFIRMED**: track_id stays stable during brief partial occlusion (bounding box visible but partly obscured); resets to new ID after full occlusion (person completely out of frame) — by design, since ByteTrack has no visual ReID and controller doesn't need ID persistence. Debug mode via `show_debug_window` shows bounding box with track_id label.
- **controller_node**: boilerplate only — declares parameters and subscribes to /target/pose, but does not yet compute or publish /cmd_vel.
- **Tests**: perception_node has 5 passing unit tests (parameter loading, publish logic with mocked camera); controller_node has 1 placeholder test.

## Known constraints
- Target tracking latency budget: aim for <150ms camera-to-cmd_vel for the
  follow behavior to look real-time.