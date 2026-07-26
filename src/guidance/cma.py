import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import cv2
import threading
import queue
import numpy as np
from time import sleep
from ultralytics import YOLO
import av  # 🔥 OpenCV FFMPEG/GStreamer 버그 및 락 충돌 우회용 라이브러리

# QoS 설정 (필요시 사용)
QOS_VEHICLE_DEFAULT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)

class VideoCapture:
    """별도의 스레드에서 카메라 프레임을 읽어 큐에 저장하는 클래스 (지연 방지)"""
    def __init__(self, source, logger):  # 💡 인자 구조를 명확히 고정
        self.logger = logger
        self.source = source
        self.q = queue.Queue(maxsize=1)
        self.running = True
        
        if isinstance(source, str) and source.isdigit():
            source = int(source)
            
        self.is_rtsp = isinstance(source, str) and source.startswith("rtsp")
        self.cap = None

        if not self.is_rtsp:
            # USB 웹캠은 기존 OpenCV 기본 백엔드로 안정적으로 오픈
            self.cap = cv2.VideoCapture(source)
            if not self.cap.isOpened():
                self.logger.error(f"❌ USB 웹캠을 열 수 없습니다: {source}")
            else:
                self.logger.info(f"✅ USB 웹캠 오픈 완료: {source}")
        else:
            self.logger.info(f"🔄 RTSP 스트림 연결 시도 중 (PyAV 백엔드): {source}")

        # 백그라운드 프레임 수집 스레드 가동
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    def _reader(self):
        if self.is_rtsp:
            # RTSP 스트림은 PyAV를 통해 저지연/노버퍼/TCP 환경으로 안전하게 디코딩
            while self.running:
                try:
                    container = av.open(self.source, options={'rtsp_transport': 'tcp', 'fflags': 'nobuffer', 'flags': 'low_delay'})
                    stream = container.streams.video[0]
                    stream.thread_type = 'AUTO' 
                    
                    self.logger.info(f"✅ RTSP 스트림 연결 성공 (PyAV)!: {self.source}")
                    
                    for frame in container.decode(stream):
                        if not self.running:
                            break
                        # PyAV 프레임을 곧바로 넘파이 BGR 배열로 변환
                        img = frame.to_ndarray(format='bgr24')
                        
                        if not self.q.empty():
                            try:
                                self.q.get_nowait()
                            except queue.Empty:
                                pass
                        self.q.put(img)
                        
                    container.close()
                except Exception as e:
                    self.logger.error(f"⚠️ RTSP 연결 오류 발생, 2초 후 재연결 시도: {e}")
                    sleep(2.0)
        else:
            # USB 웹캠 리더 루프
            while self.running:
                if self.cap is None or not self.cap.isOpened():
                    sleep(0.1)
                    continue
                ret, frame = self.cap.read()
                if not ret:
                    sleep(0.01)
                    continue
                if not self.q.empty():
                    try:
                        self.q.get_nowait()
                    except queue.Empty:
                        pass
                self.q.put(frame)

    def read(self):
        try:
            return True, self.q.get(timeout=0.1)
        except queue.Empty:
            return False, None
    
    def release(self):
        self.running = False
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()


class CameraSwitcher(Node):
    def __init__(self):
        super().__init__('camera_switcher')
        self.bridge = CvBridge()
        
        self.declare_parameter("ocam_dev", 0)
        self.declare_parameter("rtsp_url", "rtsp://192.168.144.25:8554/main.264") 
        self.declare_parameter("fps", 30.0)

        ocam_dev = self.get_parameter("ocam_dev").value
        rtsp_url = self.get_parameter("rtsp_url").get_parameter_value().string_value

        # 💡 VideoCapture 생성 시 인자 전달 방식 매칭 완료
        self._cap_usb = VideoCapture(ocam_dev, self.get_logger())     
        
        if isinstance(rtsp_url, str) and rtsp_url.isdigit():
            self._cap_eth = VideoCapture(int(rtsp_url), self.get_logger())
        else:
            self._cap_eth = VideoCapture(rtsp_url, self.get_logger())
        
        self.declare_parameter('model_path_tray', '/home/rohang/siyi_sdk/vision_detection_tray/runs_3/runs_3/detect/tray_yolo26n/weights/best.pt')
        self.declare_parameter('model_path_vertiport', '/home/rohang/siyi_sdk/vision_detection_vertiport/runs/detect/train/weights2/best.engine')

        MODEL_PATH = {
            'tray': self.get_parameter('model_path_tray').get_parameter_value().string_value,
            'vertiport': self.get_parameter('model_path_vertiport').get_parameter_value().string_value,
        }

        self.current_mission_state = "idle"
        self.current_model = None
        self.model_paths = MODEL_PATH

        self.create_subscription(String, '/mission/state', self.mission_state_callback, 10)
        
        self.usb_center_pub = self.create_publisher(Point, '/camera/usb_yolo_center', 10) 
        self.eth_center_pub = self.create_publisher(Point, '/camera/eth_yolo_center', 10)

        self.frame_count = 0 

        fps = float(self.get_parameter("fps").value)
        self.create_timer(1.0 / max(1.0, fps), self.detection_loop)

    def detection_loop(self):
        ret_usb, frame_usb = self._cap_usb.read()
        ret_eth, frame_eth = self._cap_eth.read()

        self.frame_count += 1

        # 홀수/짝수 프레임 스위칭 논리 유지
        run_usb_yolo = (self.frame_count % 2 == 1)
        run_eth_yolo = (self.frame_count % 2 == 0)

        if ret_usb:
            self.camera_stream(frame_usb, self.usb_center_pub, run_yolo=run_usb_yolo)
        if ret_eth:
            self.camera_stream(frame_eth, self.eth_center_pub, run_yolo=run_eth_yolo)

    def camera_stream(self, frame, center_pub, run_yolo):
        if frame is None:
            return
        
        if run_yolo and (self.current_model is not None):
            results = self.current_model(frame, verbose=False)[0]
            self.publish_target(results.boxes, center_pub)

    def publish_target(self, boxes, center_pub):
        if len(boxes) == 0:
            return
            
        target_box = self.get_best_conf(boxes)
        if target_box is None:
            return
            
        target_info = self.get_target_info(target_box)
        if target_info is None:
            return

        _, _, _, _, measured_cx, measured_cy = target_info
    
        point_msg = Point(x=float(measured_cx), y=float(measured_cy), z=0.0)
        center_pub.publish(point_msg)

    def mission_state_callback(self, msg):
        new_state = msg.data.lower().strip()
        if new_state != self.current_mission_state:
            self.get_logger().info(f"🔄 미션 상태 변경: '{self.current_mission_state}' -> '{new_state}'")
            self.current_mission_state = new_state
            self.load_model_for_state(new_state)
            
    def load_model_for_state(self, state):
        if state in self.model_paths:
            # 기존 모델 및 TensorRT 가속 메모리 해제
            if self.current_model is not None:
                del self.current_model
                self.current_model = None
                import gc
                gc.collect()
                try:
                    import torch
                    if torch.cuda.is_available():
                        torch.cuda.empty_cache()
                except ImportError:
                    pass

            # 신규 모델 로드 (PT 파일 혹은 TensorRT .engine 파일 자동 처리)
            self.current_model = YOLO(self.model_paths[state])
            
            # TensorRT 초기 딜레이 방지를 위한 가속 엔진 웜업
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
            self.current_model(dummy_img, verbose=False) 
            
            self.get_logger().info(f"✅ '{state}' 엔진 로드 및 웜업 완료!")
        else:
            self.current_model = None
            if state != 'idle':
                self.get_logger().warn(f"'{state}'에 해당하는 모델 경로가 없습니다.")

    def get_best_conf(self, boxes, conf_threshold=0.8):
        max_conf = conf_threshold
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
    

def main():
    rclpy.init()
    node = CameraSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, '_cap_usb') and hasattr(node._cap_usb, 'release'):
            node._cap_usb.release()
        if hasattr(node, '_cap_eth') and hasattr(node._cap_eth, 'release'):
            node._cap_eth.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()