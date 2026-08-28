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
                                                     │ (geometry_msgs/Twist)
                                                     ▼
                                            ┌──────────────────┐
                                            │  TurtleBot3      │
                                            │  (Gazebo Sim)    │
                                            └──────────────────┘
```

- `perception_node` opens the webcam directly via `cv2.VideoCapture(camera_index)` — no separate camera driver node or `/camera/image_raw` topic.
- `/target/pose` uses `visual_teleop_msgs/TrackedTarget` (2D normalized image coordinates + metadata), NOT `geometry_msgs/PoseStamped`.

## Node Descriptions

### perception_node
- **Input**: None (opens webcam directly via `cv2.VideoCapture`)
- **Processing** (current dummy implementation):
  1. OpenCV VideoCapture reads frames from `/dev/video{camera_index}`
  2. Timer callback at `publish_rate_hz` reads a frame
  3. Publishes dummy `TrackedTarget` at frame center (x=0.5, y=0.5)
- **Processing** (planned):
  1. OpenCV VideoCapture reads frames
  2. YOLOv8 detects objects in frame
  3. ByteTrack associates detections across frames
  4. Filter tracks by `target_class` parameter
  5. Select best track (highest confidence, closest to center)
  6. Publish `TrackedTarget` with normalized bbox center (x, y in 0..1), confidence, track_id
- **Output**: `/target/pose` (visual_teleop_msgs/TrackedTarget)

### controller_node
- **Input**: `/target/pose` (visual_teleop_msgs/TrackedTarget)
- **Processing** (not yet implemented):
  1. Extract target position (x, y normalized, confidence, target_visible, track_id)
  2. Compute angular error from image center (x - 0.5)
  3. Estimate distance from confidence/bbox size (or use fixed target_distance)
  4. Apply proportional control for linear/angular velocity
  5. Clamp to `max_linear_speed` / `max_angular_speed`
  6. Apply deadband
  7. Safety: publish zero velocity if `target_visible=false` for > `lost_target_timeout`
- **Output**: `/cmd_vel` (geometry_msgs/Twist)

## Data Flow Timing
Target latency budget: **<150ms camera-to-cmd_vel** (estimated, not yet measured)

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
| publish_rate_hz | double | 30.0 | Timer callback rate (Hz) |
| yolo_model | string | "yolov8n.pt" | YOLO model file (planned) |
| target_class | string | "person" | COCO class to track (planned) |
| confidence_threshold | double | 0.5 | Detection threshold (planned) |
| iou_threshold | double | 0.45 | NMS IoU threshold (planned) |
| device | string | "cpu" | Inference device: "cpu" or "cuda" (planned) |
| tracking_max_age | int | 30 | Max frames to keep lost track (planned) |
| tracking_min_hits | int | 3 | Min hits to confirm track (planned) |
| tracking_iou_threshold | double | 0.3 | Track association IoU (planned) |
| publish_annotated_image | bool | true | Publish /perception/annotated image (planned) |

### controller_node
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| linear_gain | double | 0.5 | P gain for linear velocity |
| angular_gain | double | 1.0 | P gain for angular velocity |
| max_linear_speed | double | 0.22 | Max linear speed (m/s) |
| max_angular_speed | double | 1.82 | Max angular speed (rad/s) |
| target_distance | double | 1.0 | Desired follow distance (meters) |
| deadband | double | 0.1 | Distance deadband (meters) |
| lost_target_timeout | double | 1.0 | Seconds before stopping if target lost |
| enable_safety_stop | bool | true | Stop robot if no target detected |
| publish_rate | double | 30.0 | Publish rate (Hz) |

**Note**: `publish_rate_hz` (perception) vs `publish_rate` (controller) — currently different param names; may unify later.

## Dependencies
- **Core**: rclpy, std_msgs, sensor_msgs, geometry_msgs, cv_bridge
- **Perception**: ultralytics (YOLO), opencv-python, supervision (ByteTrack) — planned
- **Messages**: visual_teleop_msgs (TrackedTarget)
- **Simulation**: turtlebot3_gazebo, gazebo_ros — planned
- **Launch**: launch, launch_ros