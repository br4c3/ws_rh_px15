#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 최종 수정 일자 
# 2026-05-08 00:01

# ================================================
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# 🎯🎯🎯추가🎯🎯🎯
from yolo_msgs.msg import DetectionArray
# 🎯🎯🎯삭제🎯🎯🎯
# from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import DistanceSensor
from time import sleep , time 
# from mission_msgs.msg import ControlTick


# from mission_common.qos import QOS_SENSOR

QOS_PUB_REL1 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10,
)
QOS_PUB_IMAGE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

QOS_VEHICLE_DEFAULT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)


class Transformation(Node):
    def __init__(self):
        super().__init__('transformation')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )



        # ---------- Intrinsics (px) ----------
        self.declare_parameter('cx', 960.0)
        self.declare_parameter('cy', 540.0)
        self.declare_parameter('fx', 1095.7)
        self.declare_parameter('fy', 1096.8)

        # self.declare_parameter('cx', 640.0)
        # self.declare_parameter('cy', 360.0)
        # self.declare_parameter('fx', 410.8)
        # self.declare_parameter('fy', 408.7)

        # self.declare_parameter('cx', 640.0)
        # self.declare_parameter('cy', 400.0)
        # self.declare_parameter('fx', 1.20501440e+03)
        # self.declare_parameter('fy', 1.20471415e+03)
        


        # CX, CY = 640.0, 360.0
        # FX, FY = 410.8, 408.7


        # ---------- Extrinsics ----------

        # self.declare_parameter('cam_offset_body', [0.342, 0.0, 0.135])
        self.declare_parameter('cam_offset_body', [0.0, 0.0, 0.0])
        self.declare_parameter('lidar_offset_body', [0.0, 0.0, 0.0])
        self.declare_parameter('ground_normal_body', [0.0, 0.0, 1.0])

        # self.declare_parameter('cam_offset_body', [0.0, 0.0, 0.0])
        # self.declare_parameter('lidar_offset_body', [0.0, 0.0, 0.0])
        # self.declare_parameter('ground_normal_body', [0.0, 0.0, 1.0])



        # 이미지 x(오른쪽) -> 바디 +Y, 이미지 y(아래) -> 바디 +X
        # self.M0_cam_to_body = np.array([
        #     [-1., 0., 0.],
        #     [0., -1., 0.],
        #     [0., 0., 1.],
        # ], dtype=float)


        self.M0_cam_to_body = np.array([
            [1., 0., 0.],
            [0., 1., 0.],
            [0., 0., 1.],
        ], dtype=float)




        # ---------- Ground / timing ----------
        self.declare_parameter('lost_timeout', 2.0)
        # self.declare_parameter('tick_topic', '/mission/control_tick')
        self.declare_parameter('output_frame_id', 'base_link')

        # ----- 허프 진입점 활성화 고도 -----
        # self.declare_parameter('hough_activation_altitude', 2.5)

        # 파라미터 로드
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        self.K_inv = np.linalg.inv(np.array([[fx, 0.0, cx], 
                                             [0.0, fy, cy], 
                                             [0.0, 0.0, 1.0]]))

        self.cam_offset_body = np.array(self.get_parameter('cam_offset_body').value)
        self.lidar_offset_body = np.array(self.get_parameter('lidar_offset_body').value)

        self.n_body = np.array(self.get_parameter('ground_normal_body').value)   #그냥 동체 수직아래 단위벡터
        n_norm = np.linalg.norm(self.n_body)
        self.n_body = self.n_body / n_norm if n_norm > 1e-6 else np.array([0.0, 0.0, 1.0]) #이미 단위 벡터인거 같은데 이거 왜해주는거지 #그니까

        self.lost_timeout = self.get_parameter('lost_timeout').value
        # tick_topic = self.get_parameter('tick_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        # self.hough_activation_alt = self.get_parameter('hough_activation_altitude').value

        # ---------- State ----------
        self.Z_lidar = None
        self.yolo_center = None
        self.aruco_center = None
        self.last_yolo_time = None
        self.last_aruco_time = None
        self.final_target = None
        # self.current_gimbal_phase = "IDLE"
        # self.phase = "IDLE"
        # self.task = "IDLE"

        self.mode = "none"

        self.pitch = 0.0
        self.yaw = 0.0
        self.last_att_time = None


        # ----- 허프 변환 관련 상태 변수 -----
        # self.hough_entry_point = None
        # self.last_hough_time = None
        # self.use_hough = False

        # ---------- Pub/Sub ----------
        # [수정] Point -> PointStamped로 구독 타입 변경 및 토픽 이름 동기화

        # self.create_subscription(PointStamped, '/yolo/detections' ,self.yolo_cb, QOS_VEHICLE_DEFAULT)
        
        # self.create_subscription(PointStamped, '/yolo/filtered_center', self.yolo_cb, 10)

        # 🎯🎯🎯수정🎯🎯🎯
        # self.create_subscription(Point, '/yolo/filtered_center', self.yolo_cb, 10)
        self.create_subscription(DetectionArray, '/yolo/detections', self.yolo_cb, 10)

        # self.create_subscription(PointStamped, '/aruco/center_px', self.aruco_cb, QOS_PUB_REL1)

        
        #실기체에서 라이더로 좌표변환
        self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', self.distance_cb, QOS_PUB_IMAGE)
        # self.create_subscription(ControlTick, tick_topic, self.tick_cb, QOS_PUB_REL1)
        # self.create_subscription(Vector3,'/gimbal/attitude', self.gimbal_atttitude, QOS_PUB_REL1) 
        # self.create_subscription(String, '/gimbal/phase', self.gimbal_phase_cb, QOS_PUB_REL1)
        # self.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position_v1',self.altitude_cb,qos_profile)

        # ----- 허프 변환 진입점 구독자 -----
        # self.create_subscription(PointStamped, '/hough/entry_point', self.hough_entry_point_cb, QOS_SUB_BEST)

        self.pub_body = self.create_publisher(PointStamped, '/target/center_point', QOS_PUB_REL1)
        # self.pub_hough = self.create_publisher(Bool, '/hough/use', QOS_PUB_REL1)
        
        self.get_logger().info('✅ [Transformation] started (ControlTick-driven).')

        dt = 1.0 / 20.0
        self.last_loop_time = time()
        self.timer = self.create_timer(dt, self.do_transformation)

    # 🎯🎯🎯수정🎯🎯🎯
    # def yolo_cb(self, msg: Point):
    #     """ [수정] YOLO 노드에서 PointStamped 메시지를 수신합니다. """
    #     self.yolo_center = np.array([msg.x, msg.y, 1.0])
    #     self.last_yolo_time = self.get_clock().now()

    # """ YOLO 노드에서 DetectionArray 메시지를 수신합니다. """
    def yolo_cb(self, msg: DetectionArray):
        # 1. 화면에 잡힌 객체가 아무것도 없으면 그냥 함수를 종료 (에러 방지)
        if not msg.detections:
            return
            
        # 2. msg 안에 들어있는 '첫 번째' 탐지 객체를 꺼냅니다.
        first_det = msg.detections[0]
        
        # 3. 데이터 구조에 맞춰 x, y 추출 (msg가 아니라 first_det에서 꺼냅니다!)
        x = first_det.bbox.center.position.x
        y = first_det.bbox.center.position.y
        
        # 4. 원하시던 np.array 형태로 저장
        self.yolo_center = np.array([x, y, 1.0])
        
        # 5. 시간 정보 저장 및 로그 출력
        self.last_yolo_time = self.get_clock().now()
        # self.last_yolo_time = rclpy.time.Time.from_msg(msg.header.stamp)



    # def aruco_cb(self, msg: PointStamped):
    #     """ [수정] ArUco 노드에서 PointStamped 메시지를 수신합니다. """
    #     self.aruco_center = np.array([msg.point.x, msg.point.y, 1.0])
    #     self.last_aruco_time = rclpy.time.Time.from_msg(msg.header.stamp)
    
    # def gimbal_atttitude(self, msg: Vector3):
    #     self.pitch = - math.radians(msg.y)
    #     self.yaw =  - math.radians(msg.x)

    def distance_cb(self, msg: DistanceSensor):
        d = float(msg.current_distance)
        if math.isfinite(d) and d > msg.min_distance:
            self.Z_lidar = d

    def _select_pixel(self):
        now = self.get_clock().now()
        def is_fresh(last_time):
            return (last_time is not None) and ((now - last_time).nanoseconds * 1e-9 <= self.lost_timeout)

        
        fresh_yolo  = (self.yolo_center  is not None) and is_fresh(self.last_yolo_time)
        # self.get_logger().info(f"aruco:{fresh_aruco}, yolo:{fresh_yolo}",throttle_duration_sec=0.5)
        if fresh_yolo:
            return ("single", self.yolo_center[:2], "yolo")



    def do_transformation(self):
        selection = self._select_pixel()

        
        if selection is None:
             
            self.get_logger().warn("No fresh YOLO data available.") 
            return 
        self.mode, *payload = selection

        # 공통적으로 필요한 체크
        if self.Z_lidar is None or self.Z_lidar == 0.0:
            self.get_logger().info("라이다 없음",throttle_duration_sec=0.5)
            return

        p_body = None
        source = None  # 항상 정의되도록 초기화

        # ---------- 모드 분기 ----------
        if self.mode == "none":
            self.get_logger().info("None",throttle_duration_sec=0.5)
            return

       

        if self.mode == "single":
            pixel_xy, source = payload  # source in {"aruco","yolo"}
        if pixel_xy is None:
            return

        alpha = 0.24
        if self.final_target is None:
            self.final_target = pixel_xy
        else:
            self.final_target = alpha * pixel_xy + (1 - alpha) * self.final_target


        
        pixel_h = np.array([self.final_target[0], self.final_target[1], 1.0])
        p_cam = self.Z_lidar * (self.K_inv @ pixel_h)
        p_body = self.M0_cam_to_body @ p_cam 
        p_tray = p_body + self.cam_offset_body     #self.M0_cam_to_body @ p_cam    카메라 (0,-90) 기준



        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = f"{self.output_frame_id}_{source}"
        out.point.x, out.point.y, out.point.z = map(float, p_tray[:3])
        self.pub_body.publish(out)
        self.get_logger().info("싱글이야", throttle_duration_sec = 1.5) 
        return


def main(args=None):
    rclpy.init(args=args)
    node = Transformation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
