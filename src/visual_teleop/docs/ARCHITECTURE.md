# Visual Teleoperation Architecture

## Overview
This package implements a visual teleoperation system where a laptop webcam tracks a target (hand, object, or face) and drives a simulated TurtleBot3 in Gazebo to follow it.

## Node Graph

```
┌──────────────────┐     /target/pose      ┌──────────────────┐
│  perception_node │ ────────────────────▶ │  controller_node │
│  (OpenCV + YOLO) │  (TrackedTarget)      │  (P Controller)  │
└──────────────────┘                        └────────┬─────────┘
                                                     │
                                                     │ /cmd_vel
                                                     │ (geometry_msgs/TwistStamped)
                                                     ▼
                                            ┌──────────────────┐
                                            │  TurtleBot3      │
                                            │  (Gazebo Sim)    │
                                            └──────────────────┘
```

- `perception_node` opens the webcam directly via `cv2.VideoCapture(camera_index)` — no separate camera driver node or `/camera/image_raw` topic.
- `/target/pose` uses `visual_teleop_msgs/TrackedTarget` (2D normalized image coordinates + metadata + timestamp), NOT `geometry_msgs/PoseStamped`.

## Node Descriptions

### perception_node
- **Input**: None (opens webcam directly via `cv2.VideoCapture`)
- **Processing**:
  1. OpenCV VideoCapture reads frames from `/dev/video{camera_index}`
  2. YOLOv8 detects objects in frame
  3. ByteTrack associates detections across frames
  4. Filter tracks by `target_class` parameter
  5. Select best track (highest confidence)
  6. Apply moving-average filter to x/y coordinates over last N frames (`smoothing_window_size`, default 5, 0=disabled)
  7. Publish `TrackedTarget` with normalized bbox center (x, y in 0..1), confidence, track_id, `target_visible` (raw per-frame), and `stamp` (frame capture time for latency measurement)
- **Output**: `/target/pose` (visual_teleop_msgs/TrackedTarget)

### controller_node
- **Input**: `/target/pose` (visual_teleop_msgs/TrackedTarget)
- **Processing**:
  1. Extract target position (x, y normalized, confidence, target_visible, track_id, stamp)
  2. Compute angular error from image center (x - 0.5)
  3. Apply proportional control for linear/angular velocity
  4. Clamp to `max_linear_speed` / `max_angular_speed`
  5. Apply dead zone (`dead_zone_px`)
  6. Safety (subscription callback + watchdog): publish zero velocity if `target_visible=false` for > `target_lost_timeout_sec` (default 1.0s). The watchdog timer (10 Hz) ensures zero Twist is published even if no new messages arrive.
  7. **Latency logging**: computes and logs time from frame capture (`msg.stamp`) to Twist publish (~every 2s)
- **Output**: `/cmd_vel` (geometry_msgs/TwistStamped)

## Data Flow Timing
Target latency budget: **<150ms camera-to-cmd_vel** (measured via `msg.stamp` → controller publish)

| Stage | Target Latency |
|-------|----------------|
| Camera capture | ~33ms (30 FPS) |
| YOLO inference | ~30-50ms (CPU, yolov8n) |
| ByteTrack update | ~5ms |
| TrackedTarget publish | ~1ms |
| Controller compute | ~1ms |
| ROS 2 publish | ~5ms |
| **Total** | **~75-95ms** |

## Parameters
All parameters in `config/params.yaml`, loaded via `declare_parameter`.

### perception_node
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| camera_index | int | 0 | Video device index (/dev/videoX) |
| camera_width | int | 640 | Capture width |
| camera_height | int | 480 | Capture height |
| camera_fps | int | 30 | Capture FPS |
| camera_backend | string | "v4l2" | Backend: v4l2, any, gstreamer, ffmpeg |
| camera_fourcc | string | "MJPG" | FourCC format: MJPG (YUYV produces corrupted frames in this env) |
| camera_warmup_frames | int | 10 | Frames to discard for auto-exposure/white-balance settle |
| yolo_model | string | "yolov8n.pt" | YOLO model file |
| target_class | string | "person" | COCO class to track |
| confidence_threshold | double | 0.5 | Detection threshold |
| iou_threshold | double | 0.45 | NMS IoU threshold |
| device | string | "cpu" | Inference device: "cpu" or "cuda" |
| track_activation_threshold | double | 0.25 | Confidence threshold for activating new tracks |
| minimum_matching_threshold | double | 0.8 | IoU threshold for matching detections to tracks |
| max_time_lost | int | 30 | Max frames to keep lost track alive |
| minimum_consecutive_frames | int | 1 | Min consecutive frames to confirm a track |
| smoothing_window_size | int | 5 | Moving average window size for x/y (0=disabled) |
| publish_annotated_image | bool | true | Publish /perception/annotated image |
| publish_rate_hz | double | 30.0 | Hz |

### controller_node
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| linear_gain | double | 0.5 | P gain for linear velocity |
| angular_gain | double | 1.0 | P gain for angular velocity |
| max_linear_speed | double | 0.22 | Max linear speed (m/s) |
| max_angular_speed | double | 1.82 | Max angular speed (rad/s) |
| target_distance | double | 1.0 | Desired follow distance (meters) |
| deadband | double | 0.1 | Distance deadband (meters) |
| dead_zone_px | double | 0.05 | Dead zone in normalized x (0-1) for angular control |
| target_lost_timeout_sec | double | 1.0 | Seconds before stopping if target lost (used by both callback check and watchdog timer) |
| enable_safety_stop | bool | true | Stop robot if no target detected |
| publish_rate_hz | double | 30.0 | Hz |

## Dependencies
- **Core**: rclpy, std_msgs, sensor_msgs, geometry_msgs, cv_bridge
- **Perception**: ultralytics (YOLO), opencv-python, supervision (ByteTrack)
- **Messages**: visual_teleop_msgs (TrackedTarget with stamp field)
- **Simulation**: turtlebot3_gazebo, gazebo_ros
- **Launch**: launch, launch_ros