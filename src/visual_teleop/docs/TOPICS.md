# Visual Teleoperation Topic Contracts

## Topic Summary

| Topic | Type | Publisher | Subscriber | Rate |
|-------|------|-----------|------------|------|
| `/target/pose` | visual_teleop_msgs/TrackedTarget | perception_node | controller_node | 30 Hz |
| `/cmd_vel` | geometry_msgs/Twist | controller_node | TurtleBot3 (Gazebo) | 30 Hz |

---

## `/target/pose`

**Publisher**: perception_node
**Subscriber**: controller_node
**QoS**: Reliable, keep last 10

```python
# visual_teleop_msgs.msg.TrackedTarget
x: float32                          # Target X in camera frame (normalized -1..1 or pixels)
y: float32                          # Target Y in camera frame (normalized -1..1 or pixels)
confidence: float32                 # Detection confidence 0.0..1.0
target_visible: bool                # True if target currently tracked
track_id: int32                     # ByteTrack track ID (persistent across frames)
```

**Coordinate Frame**: Camera image frame (origin top-left, X right, Y down)
- `x`, `y`: pixel coordinates of tracked target center (0..width, 0..height)
- OR normalized coordinates (-1..1) if `normalize_coords` param is true
- `confidence`: YOLO detection confidence for this track
- `target_visible`: False when target lost (track expired)
- `track_id`: ByteTrack-assigned persistent track ID

**Special Values**:
- If no target detected: publish with `target_visible: false`, `track_id: -1`
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