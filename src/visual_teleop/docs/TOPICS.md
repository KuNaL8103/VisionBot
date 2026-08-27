# Visual Teleoperation Topic Contracts

## Topic Summary

| Topic | Type | Publisher | Subscriber | Rate |
|-------|------|-----------|------------|------|
| `/camera/image_raw` | sensor_msgs/Image | camera driver | perception_node | 30 Hz |
| `/target/pose` | geometry_msgs/PoseStamped | perception_node | controller_node | 30 Hz |
| `/cmd_vel` | geometry_msgs/Twist | controller_node | TurtleBot3 (Gazebo) | 30 Hz |
| `/perception/annotated` | sensor_msgs/Image | perception_node | (visualization) | 30 Hz |

---

## `/camera/image_raw`

**Publisher**: Camera driver (v4l2_camera or similar)
**Subscriber**: perception_node
**QoS**: Sensor Data (best effort, keep last 1)

```python
# sensor_msgs.msg.Image
header:
  stamp: builtin_interfaces.msg.Time    # Acquisition timestamp
  frame_id: "camera_frame"              # Optical frame
height: 480
width: 640
encoding: "bgr8"                        # OpenCV default
is_bigendian: 0
step: 1920                              # width * 3 (BGR)
data: uint8[]                           # Raw pixel data
```

**Notes**:
- Frame ID must match camera calibration if used
- Encoding assumed to be `bgr8` (OpenCV default)
- If using different encoding, update perception_node accordingly

---

## `/target/pose`

**Publisher**: perception_node
**Subscriber**: controller_node
**QoS**: Reliable, keep last 10

```python
# geometry_msgs.msg.PoseStamped
header:
  stamp: builtin_interfaces.msg.Time    # Detection timestamp
  frame_id: "camera_frame"              # Camera optical frame
pose:
  position:
    x: float64                          # Target X in camera frame (meters)
    y: float64                          # Target Y in camera frame (meters)
    z: float64                          # Target Z (estimated depth, meters)
  orientation:
    x: 0.0                              # Not used (identity quaternion)
    y: 0.0
    z: 0.0
    w: 1.0
```

**Coordinate Frame**: Camera optical frame (Z forward, X right, Y down)
- Origin at camera optical center
- X: right (+), Y: down (+), Z: forward (+)

**Depth Estimation**: Simple pinhole model
```
z = (focal_length * real_height) / bbox_height
x = (cx - bbox_center_x) * z / focal_length
y = (bbox_center_y - cy) * z / focal_length
```
Where `focal_length`, `cx`, `cy` from camera intrinsics (or approximated).

**Special Values**:
- If no target detected: **do not publish** (controller handles timeout)
- If multiple targets: publish best track (highest confidence)

---

## `/cmd_vel`

**Publisher**: controller_node
**Subscriber**: TurtleBot3 (Gazebo) / diff_drive_controller
**QoS**: Reliable, keep last 10

```python
# geometry_msgs.msg.Twist
linear:
  x: float64                            # Forward velocity (m/s)
  y: 0.0                                # Unused (differential drive)
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: float64                            # Yaw rate (rad/s)
```

**Limits** (TurtleBot3 Waffle):
- `linear.x`: [-0.22, 0.22] m/s
- `angular.z`: [-1.82, 1.82] rad/s

**Behavior**:
- Positive `linear.x`: forward
- Positive `angular.z`: counter-clockwise (left turn)
- Zero velocities when target lost > `lost_target_timeout`

---

## `/perception/annotated` (Optional)

**Publisher**: perception_node (if `publish_annotated_image: true`)
**Subscriber**: rqt_image_view, web UI, etc.
**QoS**: Sensor Data (best effort, keep last 1)

```python
# sensor_msgs.msg.Image (same format as /camera/image_raw)
# Contains original frame with:
# - Bounding boxes around detected targets
# - Track IDs
# - Confidence scores
# - Class labels
```

---

## TF Frames

| Frame | Parent | Description |
|-------|--------|-------------|
| `camera_frame` | (none) | Camera optical frame |
| `base_link` | `odom` | Robot base frame (from Gazebo) |
| `odom` | (none) | Odometry frame (from Gazebo) |

**Required Transforms**:
- `camera_frame` → `base_link`: Static transform (camera mount position)
- Published by: URDF or static_transform_publisher

---

## Message Contract Rules

1. **Do not change topic names** without updating both ARCHITECTURE.md and this file
2. **Do not change message types** without updating all publishers/subscribers
3. **Timestamp synchronization**: perception_node should use frame acquisition time, not processing time
4. **Frame IDs**: Must be consistent across all nodes
5. **QoS**: Use sensor data QoS for high-rate image topics, reliable for control topics