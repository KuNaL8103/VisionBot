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
- **controller_node**: computes real Twist commands from TrackedTarget input. Subscribes to `/target/pose`, computes horizontal error `(target.x - 0.5)`, applies dead zone (`dead_zone_px`), maps error to angular velocity using `angular_vel = -error_x * angular_gain` (clamped to `max_angular_speed`). **Sign convention (ROS standard)**: target on LEFT (x < 0.5) → positive angular.z (turn LEFT/CCW); target on RIGHT (x > 0.5) → negative angular.z (turn RIGHT/CW). Sets linear velocity using `linear_gain` (clamped to `max_linear_speed`) when target is visible and within forward-facing range (`abs(error_x) < 0.3`). Publishes zero Twist when `target_visible=false` or target lost timeout exceeded. All gains/thresholds loaded from `config/params.yaml` under `controller_node`.
- **TurtleBot3 Gazebo sim**: `sim_turtlebot.launch.py` launches TurtleBot3 **Burger** model in the standard **empty_world** Gazebo simulation. The `/cmd_vel` topic (geometry_msgs/TwistStamped) is bridged via ros_gz_bridge. **Manual driving confirmed**: publishing TwistStamped to `/cmd_vel` produces correct robot motion (verified via `/odom` showing matching linear velocity).
- **Tests**: **ALL 15 tests pass** (5 perception_node + 10 controller_node). Perception tests mock cv2.VideoCapture, YOLO, and ByteTrack — no hardware required. Controller tests mock TrackedTarget inputs covering: dead-center → near-zero angular; far-left → positive angular (turn LEFT); far-right → negative angular (turn RIGHT); target_visible=false → zero Twist; dead zone; clamping; forward-facing range logic.

## Known constraints
- Target tracking latency budget: aim for <150ms camera-to-cmd_vel for the
  follow behavior to look real-time.