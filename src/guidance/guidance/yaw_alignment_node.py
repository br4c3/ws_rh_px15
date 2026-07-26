#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
# from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from yolo_msgs.msg import DetectionArray


class YawAlignmentNode(Node):
    def __init__(self):
        super().__init__("yaw_alignment_node")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # self.declare_parameter("image_width", 1280)
        # self.declare_parameter("image_height", 720)
        self.declare_parameter("target_class_name", "My-First-Project")
        self.declare_parameter("yolo_topic", "/yolo/detections")
        self.declare_parameter("image_topic", "/detection_image")
        # self.declare_parameter("odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("yaw_error_topic", "/landing/yaw_error_deg")
        self.declare_parameter("target_heading_topic", "/landing/yaw_target_heading_deg")
        self.declare_parameter("aligned_topic", "/landing/yaw_aligned")
        self.declare_parameter("sample_count", 5)
        self.declare_parameter("deadzone_deg", 5.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("min_bbox_size_px", 6)
        self.declare_parameter("morph_kernel_size", 3)
        # self.declare_parameter("min_contour_area_ratio", 0.12)
        # self.declare_parameter("max_contour_area_ratio", 0.98)

        # self.image_width = int(self.get_parameter("image_width").value)
        # self.image_height = int(self.get_parameter("image_height").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value)
        self.sample_count = int(self.get_parameter("sample_count").value)
        self.deadzone_deg = float(self.get_parameter("deadzone_deg").value)
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)
        self.min_bbox_size_px = int(self.get_parameter("min_bbox_size_px").value)
        self.morph_kernel_size = int(self.get_parameter("morph_kernel_size").value)
        # self.min_contour_area_ratio = float(self.get_parameter("min_contour_area_ratio").value)
        # self.max_contour_area_ratio = float(self.get_parameter("max_contour_area_ratio").value)

        yolo_topic = self.get_parameter("yolo_topic").value
        image_topic = self.get_parameter("image_topic").value
        # odometry_topic = self.get_parameter("odometry_topic").value
        yaw_error_topic = self.get_parameter("yaw_error_topic").value
        # target_heading_topic = self.get_parameter("target_heading_topic").value
        aligned_topic = self.get_parameter("aligned_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.bridge = CvBridge()
        self.current_frame = None
        # self.current_drone_yaw = 0.0
        self.target_drone_heading = math.nan
        self.raw_angles = []
        self.last_valid_detection_time = None

        
        #가이던스로 넘겨주는 토픽
        self.yaw_error_pub = self.create_publisher(Float32, yaw_error_topic, 10)
      
        self.create_subscription(DetectionArray, yolo_topic, self.detection_callback, 10)
        self.create_subscription(Image, image_topic, self.image_callback, 10)

        # 🎯🎯🎯추가🎯🎯🎯
        self.final_target_yaw = None
        self.alpha = 0.3
     

        self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.publish_alignment)

        self.get_logger().info(
            "Yaw alignment node started: "
            f"detections={yolo_topic}, image={image_topic}, "
            f"yaw_error={yaw_error_topic}"
        )

    def image_callback(self, msg: Image):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")


    def detection_callback(self, msg: DetectionArray):
        if self.current_frame is None:
            return

        target_detection = self.select_target_detection(msg)
        if target_detection is None:
            return

        try:
            raw_error_angle = self.extract_raw_error_angle(target_detection)
        except Exception as exc:
            self.get_logger().error(f"Yaw angle extraction failed: {exc}")
            return

        if raw_error_angle is None:
            return

        self.last_valid_detection_time = self.get_clock().now().nanoseconds / 1e9
        self.raw_angles.append(raw_error_angle)
        # self.get_logger().info(
        #     # f"[yaw sample] {len(self.raw_angles)}/{self.sample_count}, "
        #     f"raw={raw_error_angle:.1f} deg",
        #     throttle_duration_sec=1.0,
        # )

        if len(self.raw_angles) < self.sample_count:
            return

        sorted_angles = sorted(self.raw_angles)
        median_error_angle = sorted_angles[len(sorted_angles) // 2]
        self.raw_angles.clear()

        if median_error_angle > 90.0:
            median_error_angle -= 180.0
        elif median_error_angle < -90.0:
            median_error_angle += 180.0

        # 🎯🎯🎯추가🎯🎯🎯
        new_measured_yaw = median_error_angle

        # --- [EMA 필터 적용] ---
        if self.final_target_yaw is None:
            self.final_target_yaw = new_measured_yaw
        else:
            # 최단 각도 차이 계산 후 alpha 적용
            angle_diff = self.normalize_angle_deg(new_measured_yaw - self.final_target_yaw)
            self.final_target_yaw = self.normalize_angle_deg(self.final_target_yaw + self.alpha * angle_diff)

        self.target_drone_heading = self.final_target_yaw

    def select_target_detection(self, msg: DetectionArray):
        target_detection = None
        largest_area = 0.0

        for detection in msg.detections:
            if detection.class_name.lower() != self.target_class_name.lower():
                continue

            bbox_area = detection.bbox.size.x * detection.bbox.size.y
            if bbox_area > largest_area:
                largest_area = bbox_area
                target_detection = detection

        return target_detection

    def extract_raw_error_angle(self, detection):
        xmin = int(detection.bbox.center.position.x - detection.bbox.size.x / 2.0)
        ymin = int(detection.bbox.center.position.y - detection.bbox.size.y / 2.0)
        xmax = int(detection.bbox.center.position.x + detection.bbox.size.x / 2.0)
        ymax = int(detection.bbox.center.position.y + detection.bbox.size.y / 2.0)

        xmin = max(0, xmin)
        ymin = max(0, ymin)
        frame_height, frame_width = self.current_frame.shape[:2]
        xmax = min(frame_width, xmax)
        ymax = min(frame_height, ymax)

        if (xmax - xmin) < self.min_bbox_size_px or (ymax - ymin) < self.min_bbox_size_px:
            return None

        crop_img = self.current_frame[ymin:ymax, xmin:xmax]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 5, 20])
        upper_green = np.array([85, 255, 255])

        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        tray_mask = cv2.bitwise_not(green_mask)

        kernel_size = max(1, self.morph_kernel_size)
        if kernel_size > 1:
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            tray_mask = cv2.morphologyEx(tray_mask, cv2.MORPH_OPEN, kernel)
            tray_mask = cv2.morphologyEx(tray_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            tray_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            # 🎯🎯🎯추가🎯🎯🎯
            # cv2.imshow("1. Crop Image", crop_img)
            # cv2.imshow("2. Green Mask", green_mask)
            # cv2.imshow("3. Tray Mask (Morph)", tray_mask)
            # cv2.waitKey(1)

            return None

        largest_contour = max(contours, key=cv2.contourArea)
    
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)

        # 🎯🎯🎯추가🎯🎯🎯
        # box_int = np.int0(box)

        edge1 = box[1] - box[0]
        edge2 = box[2] - box[1]
        short_edge = edge1 if np.linalg.norm(edge1) < np.linalg.norm(edge2) else edge2

        # 🎯🎯🎯추가🎯🎯🎯
        # result_img = crop_img.copy()
        # cv2.drawContours(result_img, [box_int], 0, (0, 0, 255), 2)  
        # cv2.imshow("1. Crop Image", crop_img)
        # cv2.imshow("2. Green Mask", green_mask)
        # cv2.imshow("3. Tray Mask (Morph)", tray_mask)
        # cv2.imshow("4. Detection Result", result_img)
        # cv2.waitKey(1)


        return float(np.degrees(np.arctan2(short_edge[0], -short_edge[1])))

    def publish_alignment(self):
        if self.is_estimate_stale():
            self.target_drone_heading = math.nan
            # 🎯🎯🎯추가🎯🎯🎯
            self.final_target_yaw = None
            self.raw_angles.clear()

        # yaw_error = self.calculate_heading_error()
        yaw_error = self.target_drone_heading
        aligned = math.isfinite(yaw_error) and abs(yaw_error) <= self.deadzone_deg



        yaw_msg = Float32()
        yaw_msg.data = float(yaw_error)
        self.get_logger().info(
            f"[yaw errr] {yaw_error:.1f} deg, ",throttle_duration_sec=1.5)
        self.yaw_error_pub.publish(yaw_msg)


    def is_estimate_stale(self):
        if self.last_valid_detection_time is None:
            return True

        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self.last_valid_detection_time) > self.stale_timeout_sec

    @staticmethod
    def normalize_angle_deg(angle):
        return (angle + 180.0) % 360.0 - 180.0


def main(args=None):
    rclpy.init(args=args)
    node = YawAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

