# VisionBot — Webcam-Driven Visual Teleoperation
 
A ROS 2 Jazzy project that uses a laptop webcam as the **only sensor** to
track a person and drive a simulated TurtleBot3 robot in Gazebo — no
physical robot hardware required.
 
```
[Webcam] → YOLOv8 detection → ByteTrack tracking → /target/pose
                                                          │
                                                          ▼
                                          Proportional controller
                                                          │
                                                          ▼
                                        /cmd_vel → TurtleBot3 (Gazebo)
```
 
Move in front of the camera, and the simulated robot turns to follow you —
in real time, entirely in simulation.
 
---
 
## Table of Contents
 
- [What This Is](#what-this-is)
- [How It Works](#how-it-works)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Running the Full System](#running-the-full-system)
- [Configuration](#configuration)
- [Message & Topic Reference](#message--topic-reference)
- [Running Tests](#running-tests)
- [Troubleshooting](#troubleshooting)
- [Known Limitations](#known-limitations)
- [Possible Extensions](#possible-extensions)
---
 
## What This Is
 
VisionBot is a visual teleoperation pipeline built entirely in software —
your laptop's built-in or USB webcam is the only "sensor" needed. It
demonstrates a full perception-to-control robotics loop:
 
1. **Perception**: A webcam feed is run through YOLOv8 to detect a person,
   and ByteTrack assigns a stable ID to that detection across frames.
2. **Control**: A proportional controller converts the target's position in
   the camera frame into linear/angular velocity commands.
3. **Simulation**: Those commands drive a simulated TurtleBot3 (Burger)
   robot inside Gazebo, following the ROS 2 standard `/cmd_vel` interface.
This is a portfolio-style project intended to demonstrate real-world ROS 2
skills: custom message design, node architecture, parameter management,
launch file composition, and integrating a computer-vision pipeline with a
robotics control loop and simulator — while working through the genuine
environment quirks of running ROS 2 + Gazebo + a webcam inside WSL2.
 
## How It Works
 
### Node Graph
 
```
┌──────────────────┐     /target/pose      ┌──────────────────┐
│  perception_node │ ────────────────────▶ │  controller_node │
│  (OpenCV + YOLO  │  (TrackedTarget)      │  (P Controller)  │
│   + ByteTrack)   │                       └────────┬─────────┘
└──────────────────┘                                 │
                                                      │ /cmd_vel
                                                      │ (TwistStamped)
                                                      ▼
                                             ┌──────────────────┐
                                             │  TurtleBot3      │
                                             │  (Gazebo Sim)    │
                                             └──────────────────┘
```
 
- **`perception_node`** opens the webcam directly via `cv2.VideoCapture`
  (no separate camera driver node). It runs YOLOv8 (`yolov8n.pt`) to detect
  the target class (default: `person`), wraps detections with ByteTrack for
  a stable `track_id` across frames, applies a moving-average smoothing
  filter to reduce jitter, and publishes a custom `TrackedTarget` message.
- **`controller_node`** subscribes to that target position, computes a
  horizontal error from image center, and applies proportional control to
  produce linear/angular velocity — clamped to TurtleBot3's real-world
  speed limits. It publishes `geometry_msgs/TwistStamped` (required by this
  project's Gazebo bridge configuration — see
  [Troubleshooting](#troubleshooting)). It also runs a watchdog timer that
  guarantees the robot stops if the target is lost, even if no new
  perception messages arrive.
- **Gazebo** runs the standard TurtleBot3 Burger model in an empty world,
  bridged to ROS 2 topics via `ros_gz_bridge`.
### Control Logic
 
- **Angular velocity**: `angular_vel = -error_x * angular_gain`, where
  `error_x = target.x - 0.5`. Per ROS convention (REP 103), positive
  `angular.z` is counter-clockwise (turn left). A target on the left side
  of the frame produces positive `angular.z` (robot turns left to face it);
  a target on the right produces negative `angular.z`.
- **Linear velocity**: moves forward at `linear_gain` (clamped to
  `max_linear_speed`) only when the target is visible and roughly centered
  (`|error_x| < 0.3`).
- **Safety stop**: zero velocity is published whenever `target_visible` is
  false, the target hasn't been seen recently, or a 10 Hz watchdog timer
  detects the target has been lost for longer than
  `target_lost_timeout_sec`.
## Project Structure
 
```
ros2_ws/
└── src/
    ├── visual_teleop/                     # Main package
    │   ├── CLAUDE.md                      # Living project-status doc
    │   ├── package.xml
    │   ├── setup.py / setup.cfg
    │   ├── visual_teleop/
    │   │   ├── perception_node.py         # Webcam → YOLO → ByteTrack → /target/pose
    │   │   ├── controller_node.py         # /target/pose → /cmd_vel
    │   │   └── utils/
    │   │       └── tracker_wrapper.py     # ByteTrack wrapper (supervision lib)
    │   ├── launch/
    │   │   ├── perception.launch.py       # Perception node standalone
    │   │   ├── sim_turtlebot.launch.py    # TurtleBot3 Gazebo sim standalone
    │   │   └── full_system.launch.py      # Everything wired together
    │   ├── config/
    │   │   └── params.yaml                # All tunable parameters
    │   ├── test/
    │   │   ├── test_perception_node.py    # Mocked unit tests (no camera needed)
    │   │   └── test_controller_node.py    # Mocked unit tests (no sim needed)
    │   └── docs/
    │       ├── ARCHITECTURE.md            # Node graph, data flow, parameters
    │       ├── TOPICS.md                  # Message/topic contracts
    │       └── TROUBLESHOOTING.md         # Environment issues & fixes
    └── visual_teleop_msgs/                # Custom message package
        └── msg/
            └── TrackedTarget.msg          # x, y, confidence, target_visible, track_id, stamp
```
 
## Prerequisites
 
- **OS**: Ubuntu 24.04 (tested on WSL2 under Windows 11, but should work
  the same on native Ubuntu)
- **ROS 2 Jazzy**
- **Python 3.12**
- **A webcam** (built-in laptop cam or USB) — if using WSL2, this needs
  USB passthrough via `usbipd-win`; see [Troubleshooting](#troubleshooting).
- **Gazebo** (Harmonic, ships with ROS 2 Jazzy) + TurtleBot3 packages
### Python Dependencies
 
```bash
pip install ultralytics opencv-python supervision --break-system-packages
```
 
- `ultralytics` — YOLOv8 object detection (auto-downloads `yolov8n.pt` on
  first run)
- `opencv-python` — webcam capture and image processing
- `supervision` — ByteTrack multi-object tracking wrapper
### ROS 2 / System Dependencies
 
```bash
sudo apt update
sudo apt install ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo
```
 
## Setup
 
1. **Clone into a ROS 2 workspace:**
```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/KuNaL8103/VisionBot.git
```
   *(Adjust folder naming if the repo doesn't already unpack into
   `visual_teleop` + `visual_teleop_msgs` — both packages should sit
   directly under `src/`.)*
 
2. **Install dependencies** (see [Prerequisites](#prerequisites) above).
3. **Build the workspace:**
```bash
   cd ~/ros2_ws
   colcon build --packages-select visual_teleop_msgs visual_teleop
   source install/setup.bash
```
 
4. **If on WSL2, attach your webcam** (see
   [Troubleshooting](#troubleshooting) for the full usbipd-win flow):
```powershell
   # In Windows PowerShell (Admin)
   usbipd list
   usbipd attach --wsl --busid <BUSID>
```
   Then confirm it's visible inside WSL2:
```bash
   ls -la /dev/video*
```
 
5. **Set the TurtleBot3 model** (required every session, or add to your
   shell profile):
```bash
   export TURTLEBOT3_MODEL=burger
```
 
## Running the Full System
 
The simplest way to run everything — Gazebo sim, perception, and
controller — together:
 
```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch visual_teleop full_system.launch.py
```
 
Wait for Gazebo to fully load (a few seconds), then stand in front of your
webcam. Moving left/right should turn the simulated robot to follow you.
 
### Running Components Individually (for debugging)
 
```bash
# Just the Gazebo sim
ros2 launch visual_teleop sim_turtlebot.launch.py
 
# Just perception, with a debug window showing detections
ros2 run visual_teleop perception_node --ros-args -p show_debug_window:=true
 
# Just the controller
ros2 run visual_teleop controller_node
```
 
> **Note**: `ros2 run` does **not** automatically load `config/params.yaml`
> — only `ros2 launch` does. When running a node standalone with `ros2 run`
> for debugging, pass parameters explicitly:
> ```bash
> ros2 run visual_teleop perception_node --ros-args --params-file \
>   install/visual_teleop/share/visual_teleop/config/params.yaml
> ```
 
### Manually Driving the Robot (sanity check)
 
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
(requires `TURTLEBOT3_MODEL=burger` set and the sim already running)
 
## Configuration
 
All tunable parameters live in `config/params.yaml`. Highlights:
 
**Perception (`perception_node`)**
| Parameter | Default | Description |
|---|---|---|
| `camera_index` | `0` | Video device index |
| `camera_fourcc` | `"MJPG"` | Camera pixel format — **do not change to YUYV**, see Troubleshooting |
| `target_class` | `"person"` | COCO class YOLO should track |
| `confidence_threshold` | `0.5` | Minimum detection confidence |
| `smoothing_window_size` | `5` | Moving-average window for x/y jitter reduction (0 = disabled) |
| `show_debug_window` | `false` | Opens an OpenCV window with bounding boxes for visual debugging |
 
**Controller (`controller_node`)**
| Parameter | Default | Description |
|---|---|---|
| `linear_gain` / `angular_gain` | `0.5` / `1.0` | Proportional control gains |
| `max_linear_speed` / `max_angular_speed` | `0.22` / `1.82` | TurtleBot3 Burger's real speed limits |
| `dead_zone_px` | `0.05` | Ignore small horizontal errors near center |
| `target_lost_timeout_sec` | `1.0` | Stop the robot if target is lost for this long |
 
See `docs/ARCHITECTURE.md` for the complete parameter reference.
 
## Message & Topic Reference
 
| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `/target/pose` | `visual_teleop_msgs/TrackedTarget` | `perception_node` → `controller_node` |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | `controller_node` → TurtleBot3 (Gazebo) |
 
**`TrackedTarget.msg`**
```
float32 x                          # normalized 0.0–1.0, target center, image left→right
float32 y                          # normalized 0.0–1.0, target center, image top→bottom
float32 confidence                 # YOLO detection confidence
bool target_visible                # false when no detection this frame
int32 track_id                     # ByteTrack ID (resets after full occlusion — see docs/TROUBLESHOOTING.md)
builtin_interfaces/Time stamp      # frame capture time, for latency measurement
```
 
Full contract details (QoS, special values, coordinate conventions) are in
`docs/TOPICS.md`.
 
## Running Tests
 
All tests are unit tests with mocked hardware — **no camera or Gazebo
required**:
 
```bash
cd ~/ros2_ws
source install/setup.bash
python3 -m pytest src/visual_teleop/test/ -v
```
 
Perception tests mock `cv2.VideoCapture`, YOLO, and ByteTrack. Controller
tests use fabricated `TrackedTarget` messages to verify control-loop logic
(sign conventions, clamping, dead zones, watchdog behavior, and latency
computation) in isolation.
 
## Troubleshooting
 
The most valuable lessons from building this project are captured in
`docs/TROUBLESHOOTING.md`. Highlights:
 
- **Solid green / corrupted webcam frames**: caused by OpenCV defaulting to
  the `YUYV` pixel format while the camera actually streams `MJPG`. Fixed
  by explicitly forcing `camera_fourcc: "MJPG"`. If you see this, don't
  trust `cap.isOpened()`/`ret==True` alone — verify with a saved frame
  (`cv2.imwrite`) that the image is actually real.
- **WSL2 webcam access**: requires `usbipd-win` passthrough from Windows;
  it does not attach automatically.
- **`ros2 run` vs `ros2 launch`**: `ros2 run` does not load
  `config/params.yaml` — use `ros2 launch` or pass `--params-file`
  explicitly.
- **`/cmd_vel` message type**: this project's `ros_gz_bridge` configuration
  only subscribes to `geometry_msgs/TwistStamped`, not plain `Twist` —
  publishing plain `Twist` will silently produce no robot motion.
- **Slow/laggy Gazebo rendering under WSL2**: often caused by falling back
  to CPU software rendering (`llvmpipe`). Check with
  `glxinfo | grep "OpenGL renderer"`; if it shows `llvmpipe`, force GPU
  rendering via Mesa's D3D12 backend:
```bash
  export GALLIUM_DRIVER=d3d12
  export MESA_D3D12_DEFAULT_ADAPTER_NAME="<your GPU name>"
```
 
See `docs/TROUBLESHOOTING.md` for full detail and additional issues
(ByteTrack ID behavior, build errors, etc).
 
## Known Limitations
 
- **ByteTrack has no visual re-identification.** `track_id` stays stable
  through brief partial occlusion, but resets to a new ID after the target
  fully leaves the frame and reappears. This doesn't affect the follow
  behavior, since the controller only depends on `x`/`y`/`target_visible`,
  not on ID persistence.
- **Simulation only.** This project has not been tested on physical
  TurtleBot3 hardware; only the Gazebo simulation.
- **Single-target tracking.** If multiple people are in frame, the
  highest-confidence detection is followed; there's no explicit
  multi-person disambiguation logic.
- **TurtleBot3 Burger's real-world top speed is genuinely slow**
  (0.22 m/s) — this is expected, not a performance bug.
## Possible Extensions
 
- **MoveIt arm variant**: reuse `/target/pose` to drive a robotic arm's
  end-effector instead of a mobile base (`arm_controller_node.py`,
  following a MoveIt demo config).
- **Appearance-based re-identification** to preserve `track_id` across full
  occlusion (e.g., a DeepSORT-style embedding model).
- **Physical hardware deployment** on a real TurtleBot3.
- **Gesture-based control** (e.g., hand gestures instead of person-position
  tracking) using MediaPipe hand landmarks.
---
 
*Built as a ROS 2 + computer vision portfolio project — real webcam,
real YOLO detection, real ByteTrack tracking, real proportional control,
real Gazebo simulation.*