#!/home/junhyeok/yolov8-env/bin/python
# ==============================================================================
# YOLOv8과 SIYI A8-mini 짐벌을 연동한 실시간 객체 추적 시스템
# ==============================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# 🎯🎯🎯삭제🎯🎯🎯
# from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray, Detection  # yaw헤딩용 토픽
import cv2
import threading, queue
import subprocess
from cv_bridge import CvBridge
import numpy as np
from time import sleep, time
from ultralytics import YOLO
from .siyi_sdk import SIYISDK

# --- 1. 백그라운드 영상 캡처 클래스 ---
class VideoCapture:
    def __init__(self, source, bufsize=2, latest_frame_callback=None):
        self.cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG, [cv2.CAP_PROP_HW_ACCELERATION, cv2.VIDEO_ACCELERATION_ANY])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, bufsize)
        self.q = queue.Queue(maxsize=1)
        # QGC 송출은 YOLO 처리와 분리한다.
        # 캡처 스레드가 새 프레임을 받을 때마다 이 콜백으로 최신 프레임만 전달한다.
        self.latest_frame_callback = latest_frame_callback
        t = threading.Thread(target=self._reader, daemon=True)
        t.start()

    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                sleep(1)
                continue

            # QGC용 큐에는 블로킹 없이 최신 카메라 프레임을 전달한다.
            # QGC 송출이 느려져도 이 캡처 스레드와 YOLO 처리는 기다리지 않는다.
            if self.latest_frame_callback is not None:
                self.latest_frame_callback(frame)

            if not self.q.empty():
                try: self.q.get_nowait()
                except queue.Empty: pass
            self.q.put(frame)

    def read(self):
        if not self.q.empty():
            return True, self.q.get()
        return False, None
    
    def release(self):
        if self.cap.isOpened():
            self.cap.release()

# --- 2. ROS2 노드 클래스 ---
class VisionTrackerNode(Node):

    def __init__(self):
        super().__init__('vision_tracker_node')
        self.get_logger().info("Vision Tracker Node - 초기화 시작...")

        # ==== 1. ROS2 통신 설정 ====
        # 🎯🎯🎯삭제🎯🎯🎯
        # self.center_point_pub = self.create_publisher(Point, '/yolo/filtered_center', 10)
        self.yaw_detection_pub = self.create_publisher(DetectionArray, '/yolo/detections', 10)  # yaw헤딩용 토픽
        self.yaw_image_pub = self.create_publisher(Image, '/siyi/image_raw', 10)  # yaw헤딩용 토픽
        self.image_pub = self.create_publisher(Image, 'detection_image', 10)
        self.bridge = CvBridge()
        self.mission_state_sub = self.create_subscription(String, '/mission/state', self.mission_state_callback, 10)

        # ==== 2. 파라미터 선언 ====
        self.declare_parameter('rtsp_url', 'rtsp://192.168.144.25:8554/main.264')
        self.declare_parameter('siyi_ip', '192.168.144.25')
        self.declare_parameter('siyi_port', 37260)
        
        self.declare_parameter('model_path_tray', '/home/rohang/siyi_sdk/vision_detection_tray/runs_3/runs_3/detect/tray_yolo26n/weights/best.engine')
        self.declare_parameter('model_path_vertiport', '/home/rohang/siyi_sdk/vision_detection_vertiport/runs/detect/train/weights2/best.engine')
        self.declare_parameter('conf_threshold', 0.7)
        
        # ==== yolo 영상 받을 qgc 설정값 ====
        self.declare_parameter('qgc_ip', '192.168.144.141')
        self.declare_parameter('qgc_video_port', 5600)
        # 현재 실기체에서 안정적으로 확인한 QGC 송출값은 그대로 유지한다.
        self.declare_parameter('qgc_video_width', 1280)
        self.declare_parameter('qgc_video_height', 720)
        self.declare_parameter('qgc_video_fps', 15) 
        self.declare_parameter('qgc_video_bitrate', 2000)

        # ==== 3. 파라미터 가져오기 및 모델 로딩 ====
        RTSP_URL = self.get_parameter('rtsp_url').get_parameter_value().string_value
        SIYI_IP = self.get_parameter('siyi_ip').get_parameter_value().string_value
        SIYI_PORT = self.get_parameter('siyi_port').get_parameter_value().integer_value

        self.qgc_ip = self.get_parameter('qgc_ip').value
        self.qgc_port = int(self.get_parameter('qgc_video_port').value)
        self.qgc_width = int(self.get_parameter('qgc_video_width').value)
        self.qgc_height = int(self.get_parameter('qgc_video_height').value)
        self.qgc_fps = int(self.get_parameter('qgc_video_fps').value)
        self.qgc_bitrate = int(self.get_parameter('qgc_video_bitrate').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)

        self.qgc_process = None
        self.last_qgc_send_time = 0.0

        # QGC 송출 전용 상태
        # 큐 크기를 1로 제한하여 송출이 밀리면 오래된 프레임을 버리고
        # 항상 가장 최근에 들어온 카메라 프레임만 유지한다.
        self.qgc_frame_queue = queue.Queue(maxsize=1)
        self.qgc_stop_event = threading.Event()
        self.qgc_thread = None
        self.qgc_dropped_frames = 0
            
        # 모델 경로들을 딕셔너리로 관리
        model_paths = {
            'tray': self.get_parameter('model_path_tray').get_parameter_value().string_value,
            'vertiport': self.get_parameter('model_path_vertiport').get_parameter_value().string_value,
        }
        self.models = {phase: YOLO(path) for phase, path in model_paths.items()}

        self.get_logger().info("✅ 모든 모델 경로 로딩 완료.")



        # ==== 4. 상태 및 제어 변수 초기화 ====
        self.current_mission_state = "idle" 
        self.current_model = None           

        self.YAW_RANGE = (-135.0, 135.0)
        self.PITCH_RANGE = (-90.0, 30.0)

        # ==== 5. 하드웨어 초기화 ====
        self.sdk = SIYISDK(server_ip=SIYI_IP, port=SIYI_PORT, debug=False)
        if not self.sdk.connect():
            self.get_logger().error("짐벌 연결 실패!")
            return
        self.get_logger().info("✅ 짐벌 연결 완료")

        yaw0, pitch0 = 0, -90
        self.sdk.requestSetAngles(yaw0, pitch0)
        self.get_logger().info(f"✅ 짐벌 초기 위치로 이동 중... ({yaw0}°, {pitch0}°)")
        sleep(2)

        self.cap = VideoCapture(
            RTSP_URL,
            bufsize=2,
        )
        if not self.cap.cap.isOpened():
            self.get_logger().error("카메라 스트림을 열 수 없습니다!")
            return
        
        #테스트 비행시에 끄기
        # QGC GStreamer 프로세스와 전용 송출 스레드를 시작한다.
        # QGC에는 YOLO 박스가 그려진 최신 영상을 송출한다.
        self.start_qgc_stream()
        self.start_qgc_worker()
        self.load_model_for_state("tray")

        # ==== 6. 메인 루프 타이머 시작 ====
        self.timer = self.create_timer(1.0 / 20.0, self.detection_loop)
        self.get_logger().info("✅ 초기화 완료. 메인 루프 시작.")

    # ========================= 메인 루프 ========================= #
    def start_qgc_stream(self):
        gst_command = [
            'gst-launch-1.0',
            '-q',

            'fdsrc',
            'fd=0',
            '!',

            'rawvideoparse',
            'format=bgr',
            f'width={self.qgc_width}',
            f'height={self.qgc_height}',
            f'framerate={self.qgc_fps}/1',
            '!',

            # GStreamer 내부에서도 인코딩이 밀리면 오래된 프레임을 버린다.
            # Python 최신 프레임 큐와 함께 QGC 영상 지연 누적을 방지한다.
            'queue',
            'max-size-buffers=1',
            'max-size-bytes=0',
            'max-size-time=0',
            'leaky=downstream',
            '!',

            'videoconvert',
            '!',

            'x264enc',
            'tune=zerolatency',
            'speed-preset=ultrafast',
            f'bitrate={self.qgc_bitrate}',
            f'key-int-max={self.qgc_fps}',
            'bframes=0',
            '!',

            'video/x-h264,profile=baseline',
            '!',

            'rtph264pay',
            'config-interval=1',
            'pt=96',
            '!',

            'udpsink',
            f'host={self.qgc_ip}',
            f'port={self.qgc_port}',
            'sync=false',
            'async=false',
        ]

        try:
            self.qgc_process = subprocess.Popen(
                gst_command,
                stdin=subprocess.PIPE
            )

            self.get_logger().info(
                f"✅ QGC 영상 송출 시작: "
                f"{self.qgc_ip}:{self.qgc_port}, "
                f"{self.qgc_width}x{self.qgc_height} "
                f"@ {self.qgc_fps} FPS"
            )

        except Exception as e:
            self.qgc_process = None
            self.get_logger().error(
                f"QGC 영상 송출기 실행 실패: {e}"
            )

    def start_qgc_worker(self):
        """QGC 파이프 쓰기만 담당하는 전용 스레드를 시작한다."""
        if self.qgc_thread is not None and self.qgc_thread.is_alive():
            return

        self.qgc_stop_event.clear()
        self.qgc_thread = threading.Thread(
            target=self.qgc_stream_worker,
            name='qgc_stream_worker',
            daemon=True,
        )
        self.qgc_thread.start()
        self.get_logger().info("✅ QGC 최신 프레임 송출 스레드 시작")

    def enqueue_latest_qgc_frame(self, frame):
        """
        QGC 큐에 최신 프레임 하나만 남긴다.

        put()으로 기다리지 않고 put_nowait()만 사용하므로,
        QGC 인코더나 네트워크가 느려져도 카메라/YOLO 처리를 막지 않는다.
        """
        if frame is None or self.qgc_stop_event.is_set():
            return

        try:
            self.qgc_frame_queue.put_nowait(frame)
            return
        except queue.Full:
            pass

        # 큐가 차 있으면 대기 중인 오래된 프레임을 제거한다.
        try:
            self.qgc_frame_queue.get_nowait()
            self.qgc_dropped_frames += 1
        except queue.Empty:
            pass

        # 방금 캡처된 최신 프레임을 넣는다.
        try:
            self.qgc_frame_queue.put_nowait(frame)
        except queue.Full:
            # 소비자 스레드와 경합해도 메인 영상 처리를 기다리게 하지 않는다.
            self.qgc_dropped_frames += 1

    def qgc_stream_worker(self):
        """
        QGC 송출 전용 작업자.

        stdin.write()가 느려지거나 막혀도 이 스레드만 영향을 받고,
        ROS 타이머의 YOLO/center 발행 경로는 계속 동작한다.
        """
        min_send_period = 1.0 / max(self.qgc_fps, 1)

        while not self.qgc_stop_event.is_set():
            try:
                frame = self.qgc_frame_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            now = time()

            # QGC 설정 FPS보다 빠르게 들어온 프레임은 보내지 않고 버린다.
            # 다음 캡처 프레임이 다시 최신 프레임으로 큐에 들어온다.
            if now - self.last_qgc_send_time < min_send_period:
                self.qgc_dropped_frames += 1
                continue

            process = self.qgc_process
            if (
                process is None
                or process.poll() is not None
                or process.stdin is None
            ):
                continue

            stream_frame = cv2.resize(
                frame,
                (self.qgc_width, self.qgc_height),
                interpolation=cv2.INTER_LINEAR,
            )
            stream_frame = np.ascontiguousarray(stream_frame)

            write_started_at = time()

            try:
                # 이 동기식 write는 QGC 전용 스레드에서만 수행한다.
                process.stdin.write(stream_frame.tobytes())
                self.last_qgc_send_time = time()

            except BrokenPipeError:
                self.get_logger().error("QGC 영상 송출 파이프가 종료되었습니다.")
                self.qgc_process = None
                continue

            except Exception as e:
                self.get_logger().error(
                    f"QGC 프레임 송출 실패: {e}",
                    throttle_duration_sec=2.0,
                )
                continue

            write_duration = time() - write_started_at
            if write_duration > 0.1:
                self.get_logger().warn(
                    f"QGC pipe write 지연: {write_duration:.3f}s, "
                    f"버린 프레임={self.qgc_dropped_frames}",
                    throttle_duration_sec=1.0,
                )

    def detection_loop(self):
        ret, frame = self.cap.read()
        if ret:
            # QGC 송출과 완전히 분리된 YOLO/center 처리 경로이다.
            # QGC stdin.write() 상태와 무관하게 이 함수는 계속 실행된다.
            self.process_yolo_and_center_detection(
                frame,
                # 🎯🎯🎯삭제🎯🎯🎯
                run_yolo=True,
            )
  
    def process_yolo_and_center_detection(self, frame, run_yolo):
        """YOLO 추론, center 발행, Yaw 토픽 및 rqt 영상 발행만 담당한다."""
        if frame is None:
            return
        
        annotated_frame = frame.copy()
        frame_header = self.make_frame_header()  # yaw헤딩용 토픽
        self.publish_yaw_image(frame, frame_header)  # yaw헤딩용 토픽

        if run_yolo and (self.current_model is not None):
            results = self.current_model(
                frame,
                conf=self.conf_threshold,
                verbose=False
            )[0]
            self.publish_yaw_detections(results.boxes, frame_header)  # yaw헤딩용 토픽
            # 🎯🎯🎯삭제🎯🎯🎯
            self.publish_target(results.boxes, annotated_frame)

        # 시험시에 gst 영상송출 끄기 
        self.enqueue_latest_qgc_frame(annotated_frame)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"rqt 전송용 이미지 발행 실패: {e}")

    def publish_target(self, boxes, annotated_frame):
        if len(boxes) == 0:
            return
            
        target_box = self.get_best_conf(boxes)
        if target_box is None:
            return
            
        target_info = self.get_target_info(target_box)
        if target_info is None:
            return

        x1, y1, x2, y2, measured_cx, measured_cy = target_info
    
        # 🎯🎯🎯삭제🎯🎯🎯
        # point_msg = Point(x=float(measured_cx), y=float(measured_cy), z=0.0)
        # center_pub.publish(point_msg)

        
        ix1, iy1, ix2, iy2 = map(int, [x1, y1, x2, y2])
        icx, icy = int(measured_cx), int(measured_cy)
        
        
        cv2.rectangle(annotated_frame, (ix1, iy1), (ix2, iy2), (0, 255, 0), 2)
       
        cv2.circle(annotated_frame, (icx, icy), 5, (0, 0, 255), -1)
        
        cv2.putText(annotated_frame, f"Tracking: {self.current_mission_state}", (ix1, iy1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def make_frame_header(self):  # yaw헤딩용 토픽
        header = Image().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "siyi_camera"
        return header

    def publish_yaw_image(self, frame, header):  # yaw헤딩용 토픽
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_msg.header = header
            self.yaw_image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"yaw 헤딩용 원본 이미지 발행 실패: {e}")

    def publish_yaw_detections(self, boxes, header):  # yaw헤딩용 토픽
        detection_array = DetectionArray()
        detection_array.header = header

        if len(boxes) == 0:
            self.yaw_detection_pub.publish(detection_array)
            return

        model_names = getattr(self.current_model, "names", {})

        for box in boxes:
            score = float(box.conf[0].item())
            if score < self.conf_threshold:
                continue

            class_id = int(box.cls[0].item())
            class_name = str(model_names.get(class_id, class_id))
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            detection = Detection()
            detection.class_id = class_id
            detection.class_name = class_name
            detection.score = score
            detection.bbox.center.position.x = float((x1 + x2) / 2.0)
            detection.bbox.center.position.y = float((y1 + y2) / 2.0)
            detection.bbox.center.theta = 0.0
            detection.bbox.size.x = float(x2 - x1)
            detection.bbox.size.y = float(y2 - y1)
            detection_array.detections.append(detection)

        self.yaw_detection_pub.publish(detection_array)

    def mission_state_callback(self, msg):
        new_state = msg.data.lower().strip()
        if new_state != self.current_mission_state:
            self.get_logger().info(f"🔄 미션 상태 변경: '{self.current_mission_state}' -> '{new_state}'")
            self.current_mission_state = new_state
            self.load_model_for_state(new_state)
            
    def load_model_for_state(self, state: str):
        model = self.models.get(state)
        if model is not self.current_model:
            if model:
                self.get_logger().info(f"✅ Swapping to '{state}' model.")
                self.current_model = model
                
            else:
                self.get_logger().warn(f"No model found for phase '{state}'. Detector paused.")
                self.current_model = None

    def get_best_conf(self, boxes):
        max_conf = self.conf_threshold
        best_box = None
        for box in boxes:
            current_conf = box.conf[0].item()
            if current_conf > max_conf:
                max_conf = current_conf
                coords = box.xyxy[0].cpu().numpy()
                best_box = (coords[0], coords[1], coords[2], coords[3])
        return best_box

    def get_target_info(self, target_box):
        x1, y1, x2, y2 = target_box
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        return x1, y1, x2, y2, cx, cy
    

def main(args=None):
    rclpy.init(args=args)
    tracker_node = VisionTrackerNode()
    try:
        rclpy.spin(tracker_node)
    except KeyboardInterrupt:
        pass
    finally:
        tracker_node.get_logger().info("Shutting down...")
        tracker_node.sdk.requestSetAngles(0, 0)
        tracker_node.sdk.disconnect()

        # 먼저 QGC 작업자에게 종료를 알린다.
        tracker_node.qgc_stop_event.set()

        qgc_process = tracker_node.qgc_process
        if qgc_process is not None:
            try:
                # GStreamer를 먼저 종료하면 막혀 있던 pipe write도 해제된다.
                qgc_process.terminate()
                qgc_process.wait(timeout=2)

            except Exception:
                try:
                    qgc_process.kill()
                except Exception:
                    pass

        if tracker_node.qgc_thread is not None:
            tracker_node.qgc_thread.join(timeout=1.0)

        tracker_node.cap.release()
        tracker_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
