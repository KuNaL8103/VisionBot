# Visual Teleoperation Architecture

## Overview
This package implements a visual teleoperation system where a laptop webcam tracks a target (hand, object, or face) and drives a simulated TurtleBot3 in Gazebo to follow it.

## Node Graph

```
┌─────────────┐     /camera/image_raw      ┌──────────────────┐
│  Camera     │ ─────────────────────────▶ │  perception_node │
│  Driver     │   (sensor_msgs/Image)      │  (YOLO+ByteTrack)│
└─────────────┘                             └────────┬─────────┘
                                                     │
                                                     │ /target/pose
                                                     │ (geometry_msgs/PoseStamped)
                                                     ▼
                                            ┌──────────────────┐
                                            │  controller_node │
                                            │  (P Controller)  │
                                            └────────┬─────────┘
                                                     │
                                                     │ /cmd_vel
                                                     │ (geometry_msgs/Twist)
                                                     ▼
                                            ┌──────────────────┐
                                            │  TurtleBot3      │
                                            │  (Gazebo Sim)    │
                                            └──────────────────┘
```

## Node Descriptions

### perception_node
- **Input**: `/camera/image_raw` (sensor_msgs/Image) from camera driver
- **Processing**:
  1. OpenCV VideoCapture reads frames from `/dev/video0`
  2. YOLOv8 detects objects in frame
  3. ByteTrack associates detections across frames
  4. Filter tracks by `target_class` parameter
  5. Select best track (highest confidence, closest to center)
  6. Convert 2D bbox center to 3D pose estimate (simple pinhole model)
- **Output**: `/target/pose` (geometry_msgs/PoseStamped)
- **Optional Output**: `/perception/annotated` (sensor_msgs/Image) with bounding boxes

### controller_node
- **Input**: `/target/pose` (geometry_msgs/PoseStamped)
- **Processing**:
  1. Extract target position (x, y, z) from pose
  2. Compute distance error: `current_distance - target_distance`
  3. Compute angular error: `atan2(target_y, target_x)`
  4. Apply proportional control:
     - `linear_vel = linear_gain * distance_error`
     - `angular_vel = angular_gain * angular_error`
  5. Clamp to `max_linear_speed` / `max_angular_speed`
  6. Apply deadband: stop if `|distance_error| < deadband`
  7. Safety: publish zero velocity if target lost > `lost_target_timeout`
- **Output**: `/cmd_vel` (geometry_msgs/Twist)

## Data Flow Timing
Target latency budget: **<150ms camera-to-cmd_vel**

| Stage | Target Latency |
|-------|----------------|
| Camera capture | ~33ms (30 FPS) |
| YOLO inference | ~30-50ms (CPU, yolov8n) |
| ByteTrack update | ~5ms |
| Pose conversion | ~1ms |
| Controller compute | ~1ms |
| ROS 2 publish | ~5ms |
| **Total** | **~75-95ms** |

## Parameters
All parameters in `config/params.yaml`, loaded via `declare_parameter`.

### perception_node
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| camera_index | int | 0 | Video device index |
| camera_width | int | 640 | Capture width |
| camera_height | int | 480 | Capture height |
| camera_fps | int | 30 | Capture FPS |
| yolo_model | string | "yolov8n.pt" | YOLO model file |
| target_class | string | "person" | COCO class to track |
| confidence_threshold | double | 0.5 | Detection threshold |
| iou_threshold | double | 0.45 | NMS IoU threshold |
| device | string | "cpu" | Inference device |
| tracking_max_age | int | 30 | Max frames to keep track |
| tracking_min_hits | int | 3 | Min hits to confirm track |
| tracking_iou_threshold | double | 0.3 | Track association IoU |
| publish_annotated_image | bool | true | Publish annotated image |
| publish_rate | double | 30.0 | Publish rate (Hz) |

### controller_node
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| linear_gain | double | 0.5 | P gain for linear vel |
| angular_gain | double | 1.0 | P gain for angular vel |
| max_linear_speed | double | 0.22 | Max linear speed (m/s) |
| max_angular_speed | double | 1.82 | Max angular speed (rad/s) |
| target_distance | double | 1.0 | Desired follow distance (m) |
| deadband | double | 0.1 | Distance deadband (m) |
| lost_target_timeout | double | 1.0 | Stop after this (s) |
| enable_safety_stop | bool | true | Stop if target lost |
| publish_rate | double | 30.0 | Publish rate (Hz) |

## Dependencies
- **Core**: rclpy, std_msgs, sensor_msgs, geometry_msgs, cv_bridge
- **Perception**: ultralytics (YOLO), opencv-python, supervision (ByteTrack)
- **Simulation**: turtlebot3_gazebo, gazebo_ros
- **Launch**: launch, launch_ros