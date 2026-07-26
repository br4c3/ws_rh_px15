#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SIYI RTSP 영상에서 mission phase에 맞는 YOLO 객체와 ArUco 마커를 탐지하는 ROS2 노드.

발행 규칙:
  1. tray phase에서는 tray YOLO 중심 좌표를 /yolo/center/detection 으로 발행
  2. vertiport_aruco phase에서는 ArUco가 보이면 ArUco 중심 좌표를 우선 발행
  3. vertiport_aruco phase에서 ArUco가 없으면 vertiport YOLO 중심 좌표를 발행

Point 메시지 사용:
  x: 중심 픽셀 u 좌표
  y: 중심 픽셀 v 좌표
  z: 소스 구분값 (0.0 = YOLO, 1.0 = ArUco)
"""

import gc
import queue
import threading
from time import sleep

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class LatestFrameCapture:
    """카메라 프레임을 백그라운드에서 계속 읽어 최신 프레임만 보관한다."""

    def __init__(self, source, logger, bufsize=2):
        self.logger = logger
        self.q = queue.Queue(maxsize=1)
        self._stop = False

        # "0"처럼 들어온 카메라 번호 문자열은 OpenCV 장치 번호로 변환한다.
        if isinstance(source, str) and source.isdigit():
            source = int(source)

        # RTSP는 FFMPEG 백엔드와 버퍼 축소를 사용해 지연을 줄인다.
        
        # 이 부분은 카메라 번호를 처리하기 위한 코드야.
        if isinstance(source, str) and source.startswith("rtsp://"):
            self.cap = cv2.VideoCapture(
                source,
                cv2.CAP_FFMPEG,
                [
                    cv2.CAP_PROP_HW_ACCELERATION,
                    cv2.VIDEO_ACCELERATION_ANY,
                ],
            )
        else:
            self.cap = cv2.VideoCapture(source)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, bufsize)
        else:
            self.logger.error(f"카메라/비디오 소스를 열 수 없습니다: {source}")

        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while not self._stop:
            if not self.cap.isOpened():
                sleep(0.2)
                continue

            ret, frame = self.cap.read()
            if not ret:
                sleep(0.02)
                continue

            # 큐에는 항상 최신 프레임 하나만 남겨 추론 지연이 누적되지 않게 한다.
            # 큐에 이미 프레임이 있으면 그 프레임은 오래된 프레임일 가능성이 있으니까 버린다.
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


class SiyiYoloArucoCenterNode(Node):
    """YOLO와 ArUco를 함께 보고 최종 중심 좌표를 발행하는 노드."""

    YOLO_SOURCE = 0.0 #yolo 결과
    ARUCO_SOURCE = 1.0 #아르코 결과
    PHASE_TRAY = "tray"
    PHASE_VERTIPORT_ARUCO = "vertiport_aruco"

    VISION_PHASES = {
        PHASE_TRAY: {
            "model": "tray",
            "use_yolo": True,
            "use_aruco": False,
        },
        PHASE_VERTIPORT_ARUCO: {
            "model": "vertiport",
            "use_yolo": True,
            "use_aruco": True,
        },
    }

    # PHASE_ALIASES = {
    #     "tray": PHASE_TRAY,
    #     "rescue": PHASE_TRAY,
    #     "rep": PHASE_TRAY,
    #     "vertiport": PHASE_VERTIPORT_ARUCO,
    #     "vertiport_aruco": PHASE_VERTIPORT_ARUCO,
    #     "aruco": PHASE_VERTIPORT_ARUCO,
    #     "takeoff": PHASE_VERTIPORT_ARUCO,
    #     "landing": PHASE_VERTIPORT_ARUCO,
    # }

    def __init__(self):
        super().__init__("siyi_yolo_aruco_center")

        # ---------- ROS 파라미터 ----------
        self.declare_parameter("video_source", "rtsp://192.168.144.25:8554/main.264")
        self.declare_parameter('model_path_tray', '/home/rohang/siyi_sdk/vision_detection_tray/runs_3/runs_3/detect/tray_yolo26n/weights/best.engine')
        self.declare_parameter('model_path_vertiport', '/home/rohang/siyi_sdk/vision_detection_vertiport/runs/detect/train/weights2/best.engine')
        self.declare_parameter("initial_model", "tray")
        self.declare_parameter("initial_phase", "tray")

        # 최종 중심좌표 발행 토픽
        self.declare_parameter("center_topic", "/yolo/center/detection")
        
        self.declare_parameter("debug_image_topic", "detection_image")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("draw_debug", True)

        # 대회 vertiport 마커: DICT_4X4_50, id=23
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        # -1이면 보이는 마커 중 가장 큰 마커를 사용하고, 0 이상이면 해당 ID만 사용한다.
        self.declare_parameter("aruco_marker_id", 23)
        self.declare_parameter("aruco_min_area", 100.0)

        video_source = self.get_parameter("video_source").value
        center_topic = self.get_parameter("center_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value
        publish_rate = float(self.get_parameter("publish_rate").value)
        
        
        #아래 값들은 다른 함수에서도 계속 사용 
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.draw_debug = bool(self.get_parameter("draw_debug").value)
        self.aruco_marker_id = int(self.get_parameter("aruco_marker_id").value)
        self.aruco_min_area = float(self.get_parameter("aruco_min_area").value)

        # ---------- ROS Pub/Sub ----------
        #OpenCV 이미지와 ROS2 Image 메시지를 변환
        self.bridge = CvBridge()
        self.center_pub = self.create_publisher(Point, center_topic, 10)
        self.image_pub = self.create_publisher(Image, debug_image_topic, 10)
        self.create_subscription(String, "/mission/state", self.mission_state_callback, 10)

        # ---------- YOLO 모델 ----------
        # mission/state가 "tray" 또는 "vertiport"로 바뀌면 해당 모델로 전환한다.
        self.model_paths = {
            "tray": self.get_parameter("model_path_tray").value,
            "vertiport": self.get_parameter("model_path_vertiport").value,
        }

        # phase만 먼저 정하고, YOLO 파일은 실제 추론이 필요할 때 하나만 로드한다.
        initial_phase = str(self.get_parameter("initial_phase").value).lower().strip()
        self.current_phase = None
        self.current_model_name = None
        self.current_model = None
        self.loaded_model_name = None
        if not self.set_vision_phase(initial_phase):
            initial_model = str(self.get_parameter("initial_model").value).lower().strip()
            if not self.set_vision_phase(initial_model):
                self.set_vision_phase(self.PHASE_TRAY)

        # ---------- ArUco 탐지기 ----------
        self.aruco_detector = self.create_aruco_detector()

        # ---------- 영상 입력 ----------
        self.cap = LatestFrameCapture(video_source, self.get_logger())
        if not self.cap.cap.isOpened():
            raise RuntimeError(f"비디오 소스를 열 수 없습니다: {video_source}")

        self.create_timer(1.0 / max(1.0, publish_rate), self.detection_loop)
        self.get_logger().info(
            f"YOLO + ArUco center node started. topic={center_topic}, source={video_source}"
        )

    # --------여기까지 검출을 수행할 준비를 끝내고 타이머를 켠 상태--------------------    

    
    # OpenCV 환경에서 사용할 수 있는 방식으로 ArUco dictionary와 detector parameter를 준비함
    def create_aruco_detector(self):
        """OpenCV 버전에 맞춰 ArUco 탐지 함수를 만든다."""
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco 모듈이 없습니다. opencv-contrib-python 설치가 필요합니다.")

        dictionary_name = str(self.get_parameter("aruco_dictionary").value)
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise RuntimeError(f"지원하지 않는 ArUco dictionary입니다: {dictionary_name}")

        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, "DetectorParameters"):
            parameters = cv2.aruco.DetectorParameters()
        else:
            parameters = cv2.aruco.DetectorParameters_create()

        # OpenCV 4.7 이상은 ArucoDetector 클래스를 사용한다.
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            return detector.detectMarkers

        # 이전 OpenCV 버전 호환.
        def detect_markers(gray):
            return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

        return detect_markers

    # 실제 메인 처리 루프/ ret opencv에서 이미지 읽어왔는지
    
    def detection_loop(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        #rqt_image_view에 보여줄 디버그 이미지
        annotated = frame.copy()

        phase = self.VISION_PHASES[self.current_phase]
        yolo_target = None
        aruco_target = None
        all_corners = None
        all_ids = None

        if phase["use_aruco"]:
            aruco_target, all_corners, all_ids = self.detect_aruco(frame)

        # vertiport+aruco phase에서는 ArUco가 안 잡힐 때만 YOLO를 fallback으로 사용한다.
        if phase["use_yolo"] and aruco_target is None:
            yolo_target = self.detect_yolo(frame)

        if self.draw_debug:
            self.draw_debug_overlay(annotated, yolo_target, aruco_target, all_corners, all_ids)

        # 핵심 우선순위: ArUco가 보이면 ArUco 중심을 발행하고, 없을 때만 YOLO 중심을 쓴다.
        if aruco_target is not None:
            cx, cy, marker_id, area = aruco_target
            self.publish_center(cx, cy, self.ARUCO_SOURCE)
            self.get_logger().info(
                f"ArUco 중심 발행: id={marker_id}, x={cx:.1f}, y={cy:.1f}, area={area:.1f}",
                throttle_duration_sec=0.5,
            )
        elif yolo_target is not None:
            x1, y1, x2, y2, score = yolo_target
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            self.publish_center(cx, cy, self.YOLO_SOURCE)
            self.get_logger().info(
                f"YOLO 중심 발행: x={cx:.1f}, y={cy:.1f}, conf={score:.2f}",
                throttle_duration_sec=0.5,
            )

        self.publish_debug_image(annotated)

    def detect_yolo(self, frame):
        """현재 선택된 YOLO 모델에서 confidence가 가장 높은 박스를 하나 고른다."""
        model = self.get_current_yolo_model()
        if model is None:
            return None

        result = model(frame, verbose=False)[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return None

        best_target = None
        best_score = self.conf_threshold
        for box in boxes:
            score = float(box.conf[0].item())
            if score < best_score:
                continue
            x1, y1, x2, y2 = box.xyxy[0].detach().cpu().tolist()
            best_target = (x1, y1, x2, y2, score)
            best_score = score

        return best_target

    def detect_aruco(self, frame):
        """ArUco 마커를 찾고 사용할 마커 하나를 선택한다."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector(gray)

        if ids is None or len(ids) == 0:
            return None, corners, ids

        selected = None
        ids_flat = ids.flatten()

        for marker_corners, marker_id in zip(corners, ids_flat):
            marker_id = int(marker_id)
            if self.aruco_marker_id >= 0 and marker_id != self.aruco_marker_id:
                continue

            pts = marker_corners.reshape(4, 2)
            area = float(cv2.contourArea(pts.astype(np.float32)))
            if area < self.aruco_min_area:
                continue

            center = pts.mean(axis=0)
            candidate = (float(center[0]), float(center[1]), marker_id, area)

            # 여러 마커가 보이면 화면에서 가장 크게 보이는 마커를 선택한다.
            if selected is None or candidate[3] > selected[3]:
                selected = candidate

        return selected, corners, ids

    def publish_center(self, cx, cy, source):
        """픽셀 중심 좌표를 /yolo/center/detection 토픽으로 발행한다."""
        msg = Point()
        msg.x = float(cx)
        msg.y = float(cy)
        msg.z = float(source) #yolo 인지 아르코인지 판단
        self.center_pub.publish(msg)

    def publish_debug_image(self, frame):
        """rqt_image_view 등에서 확인할 수 있도록 디버그 영상을 발행한다."""
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"디버그 이미지 발행 실패: {exc}")

    def draw_debug_overlay(self, frame, yolo_target, aruco_target, all_corners, all_ids):
        """YOLO 박스, ArUco 마커, 최종 발행 중심점을 화면에 표시한다."""
        if yolo_target is not None:
            x1, y1, x2, y2, score = yolo_target
            ix1, iy1, ix2, iy2 = map(int, [x1, y1, x2, y2])
            cx = int(0.5 * (x1 + x2))
            cy = int(0.5 * (y1 + y2))
            cv2.rectangle(frame, (ix1, iy1), (ix2, iy2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(
                frame,
                f"YOLO {score:.2f}",
                (ix1, max(20, iy1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        if all_ids is not None and len(all_ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, all_corners, all_ids)

        if aruco_target is not None:
            cx, cy, marker_id, _ = aruco_target
            icx, icy = int(cx), int(cy)
            cv2.circle(frame, (icx, icy), 7, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"ARUCO id={marker_id}",
                (icx + 10, max(20, icy - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

        # 최종 선택 우선순위를 영상에 남긴다.
        if aruco_target is not None:
            label = "PUBLISH: ARUCO"
            color = (0, 0, 255)
        elif yolo_target is not None:
            label = "PUBLISH: YOLO"
            color = (0, 255, 0)
        else:
            label = "PUBLISH: NONE"
            color = (120, 120, 120)

        cv2.putText(frame, label, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    def normalize_phase_name(self, phase_name):
        """mission/state 문자열을 내부 vision phase 이름으로 바꾼다."""
        phase_name = phase_name.lower().strip()
        return self.PHASE_ALIASES.get(phase_name)

    def set_vision_phase(self, phase_name):
        """phase에 맞춰 사용할 detector 구성을 바꾼다."""
        requested_phase = phase_name
        phase_name = self.normalize_phase_name(requested_phase)
        if phase_name is None:
            self.get_logger().warn(f"지원하지 않는 vision phase입니다: {requested_phase}")
            return False

        phase = self.VISION_PHASES[phase_name]
        model_name = phase["model"]
        if model_name is not None and model_name not in self.model_paths:
            self.get_logger().warn(f"'{model_name}'에 해당하는 YOLO 모델 경로가 없습니다.")
            return False

        if phase_name == self.current_phase:
            return True

        previous_model_name = self.current_model_name
        self.current_phase = phase_name
        self.current_model_name = model_name

        if previous_model_name != model_name:
            self.unload_current_yolo_model()

        self.get_logger().info(
            f"Vision phase 전환: {phase_name} "
            f"(yolo={phase['use_yolo']}, aruco={phase['use_aruco']})"
        )
        return True

    def get_current_yolo_model(self):
        """현재 phase에서 필요한 YOLO 모델 하나만 lazy-load한다."""
        if self.current_model_name is None:
            return None

        if self.current_model is not None and self.loaded_model_name == self.current_model_name:
            return self.current_model

        path = self.model_paths.get(self.current_model_name)
        if path is None:
            self.get_logger().warn(f"'{self.current_model_name}'에 해당하는 YOLO 모델 경로가 없습니다.")
            return None

        self.unload_current_yolo_model()
        self.get_logger().info(f"YOLO 모델 로드: {self.current_model_name}")
        self.current_model = YOLO(path)
        self.loaded_model_name = self.current_model_name
        return self.current_model

    def unload_current_yolo_model(self):
        """현재 YOLO 모델 참조를 제거해 다음 phase 모델만 메모리에 남게 한다."""
        if self.current_model is None:
            self.loaded_model_name = None
            return

        self.get_logger().info(f"YOLO 모델 언로드: {self.loaded_model_name}")
        self.current_model = None
        self.loaded_model_name = None
        gc.collect()

        try:
            import torch

            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception as exc:
            self.get_logger().debug(f"CUDA cache 정리 생략: {exc}")

    def mission_state_callback(self, msg):
        """미션 상태 문자열에 맞춰 vision phase를 전환한다."""
        new_state = msg.data.lower().strip()
        self.set_vision_phase(new_state)


def main(args=None):
    rclpy.init(args=args)
    node = SiyiYoloArucoCenterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.get_logger().info("YOLO + ArUco center node shutting down...")
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
