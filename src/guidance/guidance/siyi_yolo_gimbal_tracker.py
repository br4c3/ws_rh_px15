#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import importlib
import math
import queue
import threading
from time import sleep

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class LatestFrameCapture:
    def __init__(self, source, bufsize=2):
        self.cap = cv2.VideoCapture(
            source,
            cv2.CAP_FFMPEG,
            [
                cv2.CAP_PROP_HW_ACCELERATION,
                cv2.VIDEO_ACCELERATION_ANY,
            ],
        )
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, bufsize)
        self.q = queue.Queue(maxsize=1)
        self._stop = False
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while not self._stop:
            ret, frame = self.cap.read()
            if not ret:
                sleep(0.2)
                continue
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        if self.q.empty():
            return False, None
        return True, self.q.get()

    def release(self):
        self._stop = True
        if self.cap.isOpened():
            self.cap.release()


class SiyiYoloGimbalTracker(Node):
    """Real-vehicle SIYI A8-mini YOLO detector and gimbal tracker.

    Publishes:
      /yolo/filtered_center: geometry_msgs/Point pixel center
      /gimbal/commanded_angles: geometry_msgs/Vector3Stamped, radians
      /gimbal/attitude: geometry_msgs/Vector3Stamped, radians if SDK feedback exists
      detection_image: sensor_msgs/Image debug image
    """

    def __init__(self):
        super().__init__("siyi_yolo_gimbal_tracker")

        self.declare_parameter("rtsp_url", "rtsp://192.168.144.25:8554/main.264")
        self.declare_parameter("siyi_ip", "192.168.144.25")
        self.declare_parameter("siyi_port", 37260)
        self.declare_parameter("siyi_sdk_module", "guidance.siyi_sdk")
        self.declare_parameter("model_path_tray", "/home/rohang/siyi_sdk/vision_detection_tray/runs_3/runs_3/detect/tray_yolo26n/weights/best.engine")
        self.declare_parameter("model_path_vertiport", "/home/rohang/siyi_sdk/vision_detection_vertiport/runs/detect/train/weights2/best.engine")
        self.declare_parameter("initial_mode", "tray")

        self.declare_parameter("center_topic", "/yolo/filtered_center")
        self.declare_parameter("commanded_angles_topic", "/gimbal/commanded_angles")
        self.declare_parameter("attitude_topic", "/gimbal/attitude")
        self.declare_parameter("debug_image_topic", "detection_image")

        # Values found in /home/y/ws_rh_px15/src/guidance/guidance/coordinate_tf.py.
        self.declare_parameter("image_width", 1920.0)
        self.declare_parameter("image_height", 1080.0)
        self.declare_parameter("cx", 960.0)
        self.declare_parameter("cy", 540.0)
        self.declare_parameter("fx", 1095.7)
        self.declare_parameter("fy", 1096.8)

        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("draw_debug", True)

        self.declare_parameter("initial_yaw_deg", 0.0)
        self.declare_parameter("initial_pitch_deg", -90.0)
        self.declare_parameter("min_yaw_deg", -135.0)
        self.declare_parameter("max_yaw_deg", 135.0)
        self.declare_parameter("min_pitch_deg", -90.0)
        self.declare_parameter("max_pitch_deg", 25.0)
        self.declare_parameter("yaw_gain", 0.1)
        self.declare_parameter("pitch_gain", 0.1)
        self.declare_parameter("yaw_direction", -1.0)
        self.declare_parameter("pitch_direction", -1.0)
        self.declare_parameter("deadband_px", 8.0)
        self.declare_parameter("max_step_deg", 2.0)
        self.declare_parameter("send_angle_command", True)

        rtsp_url = self.get_parameter("rtsp_url").value
        siyi_ip = self.get_parameter("siyi_ip").value
        siyi_port = int(self.get_parameter("siyi_port").value)
        sdk_module = self.get_parameter("siyi_sdk_module").value

        self.center_topic = self.get_parameter("center_topic").value
        self.commanded_angles_topic = self.get_parameter("commanded_angles_topic").value
        self.attitude_topic = self.get_parameter("attitude_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value

        self.image_width = float(self.get_parameter("image_width").value)
        self.image_height = float(self.get_parameter("image_height").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.draw_debug = bool(self.get_parameter("draw_debug").value)

        self.yaw_deg = float(self.get_parameter("initial_yaw_deg").value)
        self.pitch_deg = float(self.get_parameter("initial_pitch_deg").value)
        self.min_yaw_deg = float(self.get_parameter("min_yaw_deg").value)
        self.max_yaw_deg = float(self.get_parameter("max_yaw_deg").value)
        self.min_pitch_deg = float(self.get_parameter("min_pitch_deg").value)
        self.max_pitch_deg = float(self.get_parameter("max_pitch_deg").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.pitch_gain = float(self.get_parameter("pitch_gain").value)
        self.yaw_direction = float(self.get_parameter("yaw_direction").value)
        self.pitch_direction = float(self.get_parameter("pitch_direction").value)
        self.deadband_px = float(self.get_parameter("deadband_px").value)
        self.max_step_deg = float(self.get_parameter("max_step_deg").value)
        self.send_angle_command = bool(self.get_parameter("send_angle_command").value)

        model_paths = {
            "tray": self.get_parameter("model_path_tray").value,
            "vertiport": self.get_parameter("model_path_vertiport").value,
        }
        self.models = {name: YOLO(path) for name, path in model_paths.items()}
        self.current_mode = str(self.get_parameter("initial_mode").value).lower()
        self.current_model = self.models.get(self.current_mode)

        sdk_cls = getattr(importlib.import_module(sdk_module), "SIYISDK")
        self.sdk = sdk_cls(server_ip=siyi_ip, port=siyi_port, debug=False)
        if not self.sdk.connect():
            raise RuntimeError("SIYI gimbal connection failed")
        self.get_logger().info("SIYI gimbal connected")

        self.sdk.requestSetAngles(self.yaw_deg, self.pitch_deg)
        sleep(1.0)

        self.cap = LatestFrameCapture(rtsp_url, bufsize=2)
        if not self.cap.cap.isOpened():
            raise RuntimeError(f"RTSP stream open failed: {rtsp_url}")

        self.bridge = CvBridge()
        self.center_pub = self.create_publisher(Point, self.center_topic, 10)
        self.angles_pub = self.create_publisher(
            Vector3Stamped,
            self.commanded_angles_topic,
            10,
        )
        self.attitude_pub = self.create_publisher(Vector3Stamped, self.attitude_topic, 10)
        self.image_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.create_subscription(String, "/mission/state", self.mission_state_cb, 10)

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / max(publish_rate, 1.0), self.timer_cb)

        self.get_logger().info(
            "siyi_yolo_gimbal_tracker started: "
            f"mode={self.current_mode}, center={self.center_topic}, "
            f"angles={self.commanded_angles_topic}, image={self.image_width:.0f}x{self.image_height:.0f}"
        )

    def mission_state_cb(self, msg: String):
        mode = msg.data.lower().strip()
        model = self.models.get(mode)
        if model is None:
            self.get_logger().warn(f"No YOLO model for mission state '{mode}'")
            return
        if mode != self.current_mode:
            self.current_mode = mode
            self.current_model = model
            self.get_logger().info(f"YOLO model switched to {mode}")

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.publish_gimbal_angles()
            return

        height, width = frame.shape[:2]
        if abs(width - self.image_width) > 1.0 or abs(height - self.image_height) > 1.0:
            self.image_width = float(width)
            self.image_height = float(height)
            self.get_logger().warn(
                f"RTSP frame size is {width}x{height}; check cx/cy/fx/fy parameters.",
                throttle_duration_sec=5.0,
            )

        annotated = frame.copy()
        if self.current_model is None:
            self.publish_debug_image(annotated)
            self.publish_gimbal_angles()
            return

        result = self.current_model(frame, verbose=False)[0]
        target = self.select_best_box(result.boxes)
        if target is None:
            self.publish_debug_image(annotated)
            self.publish_gimbal_angles()
            self.get_logger().warn("No valid YOLO target; holding gimbal command.", throttle_duration_sec=1.0)
            return

        x1, y1, x2, y2, score = target
        u = 0.5 * (x1 + x2)
        v = 0.5 * (y1 + y2)

        point = Point()
        point.x = float(u)
        point.y = float(v)
        point.z = float(score)
        self.center_pub.publish(point)

        self.update_gimbal_command(u, v)
        self.publish_gimbal_angles()

        if self.draw_debug:
            self.draw_target(annotated, x1, y1, x2, y2, u, v, score)
        self.publish_debug_image(annotated)

    def select_best_box(self, boxes):
        if boxes is None or len(boxes) == 0:
            return None
        best = None
        best_score = self.conf_threshold
        for box in boxes:
            score = float(box.conf[0].item())
            if score < best_score:
                continue
            x1, y1, x2, y2 = box.xyxy[0].detach().cpu().tolist()
            best = (x1, y1, x2, y2, score)
            best_score = score
        return best

    def update_gimbal_command(self, u, v):
        err_u = self.apply_deadband(float(u) - self.cx)
        err_v = self.apply_deadband(float(v) - self.cy)

        yaw_error_deg = math.degrees(math.atan2(err_u, self.fx))
        pitch_error_deg = math.degrees(math.atan2(err_v, self.fy))
        yaw_step = self.clamp(
            self.yaw_direction * self.yaw_gain * yaw_error_deg,
            -self.max_step_deg,
            self.max_step_deg,
        )
        pitch_step = self.clamp(
            self.pitch_direction * self.pitch_gain * pitch_error_deg,
            -self.max_step_deg,
            self.max_step_deg,
        )

        self.yaw_deg = self.clamp(self.yaw_deg + yaw_step, self.min_yaw_deg, self.max_yaw_deg)
        self.pitch_deg = self.clamp(
            self.pitch_deg + pitch_step,
            self.min_pitch_deg,
            self.max_pitch_deg,
        )

        if self.send_angle_command:
            self.sdk.requestSetAngles(self.yaw_deg, self.pitch_deg)

        self.get_logger().info(
            "siyi track: "
            f"err=({err_u:.1f}, {err_v:.1f}) px, "
            f"cmd yaw={self.yaw_deg:.1f} deg, pitch={self.pitch_deg:.1f} deg",
            throttle_duration_sec=0.5,
        )

    def publish_gimbal_angles(self):
        now = self.get_clock().now().to_msg()

        commanded = Vector3Stamped()
        commanded.header.stamp = now
        commanded.header.frame_id = "siyi_command"
        commanded.vector.x = 0.0
        commanded.vector.y = math.radians(self.pitch_deg)
        commanded.vector.z = math.radians(self.yaw_deg)
        self.angles_pub.publish(commanded)

        try:
            yaw_deg, pitch_deg, roll_deg = self.sdk.getAttitude()
            attitude = Vector3Stamped()
            attitude.header.stamp = now
            attitude.header.frame_id = "siyi_feedback"
            attitude.vector.x = math.radians(float(roll_deg))
            attitude.vector.y = math.radians(float(pitch_deg))
            attitude.vector.z = math.radians(float(yaw_deg))
            self.attitude_pub.publish(attitude)
        except Exception as exc:
            self.get_logger().warn(
                f"SIYI attitude feedback publish failed: {exc}",
                throttle_duration_sec=2.0,
            )

    def draw_target(self, image, x1, y1, x2, y2, u, v, score):
        ix1, iy1, ix2, iy2 = map(int, [x1, y1, x2, y2])
        cv2.rectangle(image, (ix1, iy1), (ix2, iy2), (0, 255, 0), 2)
        cv2.circle(image, (int(u), int(v)), 5, (0, 0, 255), -1)
        cv2.drawMarker(
            image,
            (int(self.cx), int(self.cy)),
            (255, 0, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
        )
        cv2.putText(
            image,
            f"{self.current_mode} {score:.2f}",
            (ix1, max(20, iy1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

    def publish_debug_image(self, image):
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.image_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(
                f"Debug image publish failed: {exc}",
                throttle_duration_sec=2.0,
            )

    def apply_deadband(self, value):
        return 0.0 if abs(value) <= self.deadband_px else value

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def destroy_node(self):
        try:
            self.sdk.requestSetAngles(0.0, 0.0)
            self.sdk.disconnect()
        except Exception:
            pass
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SiyiYoloGimbalTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
