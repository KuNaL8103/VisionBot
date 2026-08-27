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
- Webcam confirmed working natively in WSL2 via /dev/video0 (no usbipd-win
  needed — OpenCV's GStreamer backend handles it directly)

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
Workspace just scaffolded. No nodes implemented yet.

## Known constraints
- Target tracking latency budget: aim for <150ms camera-to-cmd_vel for the
  follow behavior to look real-time.