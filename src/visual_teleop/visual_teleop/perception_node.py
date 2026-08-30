#!/usr/bin/env python3
"""
Perception Node - YOLO Detection + ByteTrack

Opens webcam using OpenCV VideoCapture, runs YOLOv8 detection on each frame,
uses ByteTrack to maintain stable track IDs across frames, publishes
TrackedTarget with normalized bbox center of highest-confidence detection
for the configured target_class.
"""

import rclpy
from rclpy.node import Node
from visual_teleop_msgs.msg import TrackedTarget
import cv2
import numpy as np
from ultralytics import YOLO
from visual_teleop.utils.tracker_wrapper import TrackerWrapper


class PerceptionNode(Node):
    """Node for publishing YOLO-detected target position."""

    def __init__(self):
        super().__init__('perception_node')

        # Declare parameters (loaded from config/params.yaml)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('camera_backend', 'v4l2')  # v4l2, any, gstreamer, ffmpeg
        self.declare_parameter('camera_fourcc', 'MJPG')   # MJPG (default — YUYV causes corrupted frames on this hardware, see TROUBLESHOOTING.md)
        self.declare_parameter('camera_warmup_frames', 10)  # Frames to discard for auto-exposure settle
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('show_debug_window', False)
        # ByteTrack parameters
        self.declare_parameter('track_activation_threshold', 0.25)
        self.declare_parameter('minimum_matching_threshold', 0.8)
        self.declare_parameter('max_time_lost', 30)
        self.declare_parameter('minimum_consecutive_frames', 1)

        # Get parameter values
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.camera_width = self.get_parameter('camera_width').get_parameter_value().integer_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().integer_value
        self.camera_fps = self.get_parameter('camera_fps').get_parameter_value().integer_value
        self.camera_backend = self.get_parameter('camera_backend').get_parameter_value().string_value
        self.camera_fourcc = self.get_parameter('camera_fourcc').get_parameter_value().string_value
        self.camera_warmup_frames = self.get_parameter('camera_warmup_frames').get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.yolo_model_path = self.get_parameter('yolo_model').get_parameter_value().string_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.show_debug_window = self.get_parameter('show_debug_window').get_parameter_value().bool_value
        # ByteTrack parameters
        self.track_activation_threshold = self.get_parameter('track_activation_threshold').get_parameter_value().double_value
        self.minimum_matching_threshold = self.get_parameter('minimum_matching_threshold').get_parameter_value().double_value
        self.max_time_lost = self.get_parameter('max_time_lost').get_parameter_value().integer_value
        self.minimum_consecutive_frames = self.get_parameter('minimum_consecutive_frames').get_parameter_value().integer_value

        self.get_logger().info(f'Perception node initialized')
        self.get_logger().info(f'  camera_index: {self.camera_index}')
        self.get_logger().info(f'  camera_width: {self.camera_width}')
        self.get_logger().info(f'  camera_height: {self.camera_height}')
        self.get_logger().info(f'  camera_fps: {self.camera_fps}')
        self.get_logger().info(f'  camera_backend: {self.camera_backend}')
        self.get_logger().info(f'  camera_fourcc: {self.camera_fourcc}')
        self.get_logger().info(f'  camera_warmup_frames: {self.camera_warmup_frames}')
        self.get_logger().info(f'  publish_rate_hz: {self.publish_rate_hz}')
        self.get_logger().info(f'  yolo_model: {self.yolo_model_path}')
        self.get_logger().info(f'  target_class: {self.target_class}')
        self.get_logger().info(f'  confidence_threshold: {self.confidence_threshold}')
        self.get_logger().info(f'  iou_threshold: {self.iou_threshold}')
        self.get_logger().info(f'  device: {self.device}')
        self.get_logger().info(f'  show_debug_window: {self.show_debug_window}')
        self.get_logger().info(f'  track_activation_threshold: {self.track_activation_threshold}')
        self.get_logger().info(f'  minimum_matching_threshold: {self.minimum_matching_threshold}')
        self.get_logger().info(f'  max_time_lost: {self.max_time_lost}')
        self.get_logger().info(f'  minimum_consecutive_frames: {self.minimum_consecutive_frames}')

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {self.yolo_model_path}')
        self.model = YOLO(self.yolo_model_path)
        self.model.to(self.device)
        # Get class names from model
        self.class_names = self.model.names
        self.get_logger().info(f'YOLO model loaded. Classes: {self.class_names}')

        # Find target class index
        self.target_class_idx = None
        for idx, name in self.class_names.items():
            if name == self.target_class:
                self.target_class_idx = idx
                break
        if self.target_class_idx is None:
            self.get_logger().warn(f'Target class "{self.target_class}" not found in model classes: {list(self.class_names.values())}')
        else:
            self.get_logger().info(f'Target class "{self.target_class}" has index {self.target_class_idx}')

        # Initialize ByteTrack tracker
        self.tracker = TrackerWrapper(
            track_activation_threshold=self.track_activation_threshold,
            minimum_matching_threshold=self.minimum_matching_threshold,
            max_time_lost=self.max_time_lost,
            minimum_consecutive_frames=self.minimum_consecutive_frames
        )
        self.get_logger().info('ByteTrack tracker initialized')

        # Open webcam with specified backend
        backend_map = {
            'v4l2': cv2.CAP_V4L2,
            'any': cv2.CAP_ANY,
            'gstreamer': cv2.CAP_GSTREAMER,
            'ffmpeg': cv2.CAP_FFMPEG,
        }
        backend = backend_map.get(self.camera_backend.lower(), cv2.CAP_V4L2)
        self.cap = cv2.VideoCapture(self.camera_index, backend)
        if not self.cap.isOpened():
            self.get_logger().warn(f'Could not open camera at index {self.camera_index} with backend {self.camera_backend}')
        else:
            # Set FourCC format
            fourcc = cv2.VideoWriter_fourcc(*self.camera_fourcc)
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
            self.get_logger().info(f'  Camera opened: {int(actual_width)}x{int(actual_height)} @ {actual_fps:.1f} FPS, FourCC: {fourcc_str}')

            # Warmup: discard frames to let auto-exposure/white-balance settle
            self.get_logger().info(f'  Warming up camera ({self.camera_warmup_frames} frames)...')
            for _ in range(self.camera_warmup_frames):
                self.cap.read()
            self.get_logger().info(f'  Camera warmup complete')

        # Publisher for target info
        self.target_pub = self.create_publisher(TrackedTarget, '/target/pose', 10)

        # State for holding last known position when target lost
        self.last_x = 0.5
        self.last_y = 0.5

        # Timer for processing loop
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

    def timer_callback(self):
        """Timer callback: read frame, run YOLO detection, update ByteTrack, publish TrackedTarget."""
        # Read frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera', throttle_duration_sec=5.0)
            self._publish_target_visible_false()
            return

        frame_height, frame_width = frame.shape[:2]

        # Run YOLO inference
        results = self.model(frame, verbose=False, conf=self.confidence_threshold, iou=self.iou_threshold, device=self.device)

        # Collect all target class detections
        boxes_list = []
        confidences_list = []
        class_ids_list = []

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                cls_idx = int(box.cls.item())
                conf = float(box.conf.item())
                # Filter by target class
                if self.target_class_idx is not None and cls_idx != self.target_class_idx:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                boxes_list.append([x1, y1, x2, y2])
                confidences_list.append(conf)
                class_ids_list.append(cls_idx)

        # Update ByteTrack tracker
        if boxes_list:
            boxes_array = np.array(boxes_list, dtype=np.float32)
            confidences_array = np.array(confidences_list, dtype=np.float32)
            class_ids_array = np.array(class_ids_list, dtype=np.int32)
        else:
            boxes_array = np.empty((0, 4), dtype=np.float32)
            confidences_array = np.empty(0, dtype=np.float32)
            class_ids_array = np.empty(0, dtype=np.int32)

        tracked_detections = self.tracker.update(boxes_array, confidences_array, class_ids_array)

        # Prepare and publish message
        msg = TrackedTarget()
        if len(tracked_detections) > 0:
            # Select best track (highest confidence)
            best_idx = np.argmax(tracked_detections.confidence)
            x1, y1, x2, y2 = tracked_detections.xyxy[best_idx]
            best_confidence = float(tracked_detections.confidence[best_idx])
            track_id = int(tracked_detections.tracker_id[best_idx]) if tracked_detections.tracker_id is not None else 0

            # Compute center in normalized coordinates (0.0-1.0, origin top-left)
            center_x = (x1 + x2) / 2.0 / frame_width
            center_y = (y1 + y2) / 2.0 / frame_height

            msg.x = float(center_x)
            msg.y = float(center_y)
            msg.confidence = best_confidence
            msg.target_visible = True
            msg.track_id = track_id

            # Update last known position
            self.last_x = msg.x
            self.last_y = msg.y

            # Debug window
            if self.show_debug_window:
                self._draw_debug_frame(frame, x1, y1, x2, y2, best_confidence, self.target_class, track_id=track_id)
        else:
            self._publish_target_visible_false(frame)

        self.target_pub.publish(msg)

    def _publish_target_visible_false(self, frame=None):
        """Publish TrackedTarget with target_visible=false, holding last known position."""
        msg = TrackedTarget()
        msg.x = self.last_x
        msg.y = self.last_y
        msg.confidence = 0.0
        msg.target_visible = False
        msg.track_id = 0
        self.target_pub.publish(msg)

        if self.show_debug_window and frame is not None:
            self._draw_debug_frame(frame, 0, 0, 0, 0, 0.0, self.target_class, no_detection=True)

    def _draw_debug_frame(self, frame, x1, y1, x2, y2, conf, class_name, no_detection=False, track_id=0):
        """Draw bounding box and info on frame for debug visualization."""
        debug_frame = frame.copy()
        if not no_detection and x2 > x1 and y2 > y1:
            # Draw bounding box
            cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            # Draw label with track_id
            label = f'{class_name} ID:{track_id}: {conf:.2f}'
            cv2.putText(debug_frame, label, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # Draw center point
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            cv2.circle(debug_frame, (center_x, center_y), 5, (0, 0, 255), -1)
        else:
            # No detection - draw "NO DETECTION" text
            cv2.putText(debug_frame, 'NO DETECTION', (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            # Draw last known position
            center_x = int(self.last_x * frame.shape[1])
            center_y = int(self.last_y * frame.shape[0])
            cv2.circle(debug_frame, (center_x, center_y), 5, (255, 0, 0), -1)
            cv2.putText(debug_frame, 'LAST KNOWN', (center_x + 10, center_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # Draw image center crosshair
        h, w = debug_frame.shape[:2]
        cv2.line(debug_frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 255, 255), 1)
        cv2.line(debug_frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 255, 255), 1)

        cv2.imshow('YOLO Detection Debug', debug_frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if self.show_debug_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()