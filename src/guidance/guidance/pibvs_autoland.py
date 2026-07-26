#!/usr/bin/env python3

import numpy as np
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray , Bool  # yaw정렬 추가
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Point
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleAttitude, VehicleLocalPosition, VehicleStatus, DistanceSensor

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


class PIDGuidanceNode(Node):
    """LOS → Velocity PID Guidance, with catch verification and climb/fallback logic"""

    def __init__(self):
        
        

        super().__init__('pid_guidance_offboard')

        # QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )


        self.declare_parameter('kp_xy_one', 0.35)
        self.declare_parameter('kp_xy_two', 0.30)
        self.declare_parameter('kp_xy_final', 0.22)

        self.declare_parameter('kp_yaw', 0.13)
        self.declare_parameter('kp_z', 0.15)

        self.declare_parameter('vel_max_one', 0.8)
        self.declare_parameter('vel_max_two', 0.5)
        self.declare_parameter('vel_max_final', 0.3)
        self.declare_parameter('vel_max_z', 0.3)
        self.declare_parameter('vel_max_yaw', 10.0)


        self.declare_parameter('land_hold_time', 0.5)
        self.declare_parameter('gate_tray_alt', 1.8) 


        self.declare_parameter('xy_tol', 0.04)
        self.declare_parameter('z_tol', 0.025)
        self.declare_parameter('yaw_tol', 5.0)

        self.declare_parameter('vz__20', 2.5)   # >10 m
        self.declare_parameter('vz_20_10', 1.0)   # 20~10 m
        self.declare_parameter('vz_10_5', 0.7)  # 10~5 m
        self.declare_parameter('vz_5_3', 0.7)   # 5~3 m
        self.declare_parameter('vz_3_0', 0.15)  # 3~0 m

        self.declare_parameter('pre_land_hold_time', 1.5)


        self.kp_xy_one = self.get_parameter('kp_xy_one').value
        self.kp_xy_two = self.get_parameter('kp_xy_two').value
        self.kp_xy_final = self.get_parameter('kp_xy_final').value

        self.kp_z = self.get_parameter('kp_z').value
        self.kp_yaw = self.get_parameter('kp_yaw').value

        self.land_hold_time = float(self.get_parameter('land_hold_time').value)
        self.pre_land_hold_time = float(self.get_parameter('pre_land_hold_time').value)
        self.gate_altitude = float(self.get_parameter('gate_tray_alt').value)

        self.xy_tol = self.get_parameter('xy_tol').value
        self.z_tol = self.get_parameter('z_tol').value
        self.yaw_tol_deg = self.get_parameter('yaw_tol').value


        self.vz__20  = self.get_parameter('vz__20').value
        self.vz_20_10  = self.get_parameter('vz_20_10').value
        self.vz_10_5 = self.get_parameter('vz_10_5').value
        self.vz_5_3  = self.get_parameter('vz_5_3').value
        self.vz_3_0  = self.get_parameter('vz_3_0').value


        self.vel_max_one = self.get_parameter('vel_max_one').value
        self.vel_max_two = self.get_parameter('vel_max_two').value
        self.vel_max_final = self.get_parameter('vel_max_final').value
        self.vel_max_z = self.get_parameter('vel_max_z').value
        self.vel_max_yaw = self.get_parameter('vel_max_yaw').value



        self.land_command_sent = False
        self.xy_aligned_since = None

        # 목표점 timeout
        self.last_target_time = None
        self.target_timeout = 0.5

        self.pre_land_hold_start = None

        self.xy_aligned_gate = False
        self.z_aligned_gate = False           # current detected area
        # self.yaw_aligned_gate = False  # yaw정렬 추가
        # self.yaw_error_deg = math.nan  # yaw정렬 추가

        # self.last_yaw_error_time = None  # yaw정렬 추가
                
        self.R_q_valid = False
    
        self.is_offboard = False

        # Pose and target
        self.drone_pos = np.array([0.0, 0.0, 0.0])
        self.target_pos = None
        self.prev_time = self.get_clock().now()


        # Publishers
        self.offboard_setpoint_counter_ = 0
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        # self.do_grip_pub = self.create_publisher(Bool, '/Gripper/grip_or_not', qos_profile)

        # Subscriptions
        self.create_subscription(PointStamped, '/target/center_point', self.target_callback, qos_profile)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.quaternion_callback, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile_sensor_data)
        #self.create_subscription(Float32, '/landing/yaw_error_deg', self.yaw_error_callback, qos_profile)  # yaw정렬 추가
        

        # Timers
        self.create_timer(0.1, self.offboard_timer_callback)
        self.create_timer(0.05, self.loop)

        # Rotation matrix
        self.R_q = np.zeros((3,3))


    def offboard_timer_callback(self):
            
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        # msg.actuator = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

        if self.offboard_setpoint_counter_ == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info('Switching to Offboard mode')
            self.is_offboard = True
            

        if self.offboard_setpoint_counter_ < 11:
            self.publish_setpoint([0.0, 0.0, 0.0])
            self.offboard_setpoint_counter_ += 1

    #land시에만 parm4=nan으로 정함
    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command

        if command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            defaults = {
                'param1': 0.0,
                'param2': 0.0,
                'param3': math.nan,
                'param4': math.nan,
                'param5': math.nan,
                'param6': math.nan,
                'param7': math.nan,
            }
        else:
            defaults = {f'param{i}': 0.0 for i in range(1, 8)}

        for i in range(1, 8):
            name = f'param{i}'
            setattr(msg, name, params.get(name, defaults[name]))

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)
    
    def quaternion_callback(self, msg: VehicleAttitude):
        q0, q1, q2, q3 = msg.q
        self.R_q = 2 * np.array([
            [q0*q0+q1*q1-0.5, q1*q2-q0*q3,   q0*q2+q1*q3],
            [q0*q3+q1*q2,     q0*q0+q2*q2-0.5, q2*q3-q0*q1],
            [q1*q3-q0*q2,     q0*q1+q2*q3,   q0*q0+q3*q3-0.5]
        ])
        self.R_q_valid = True


    def target_callback(self, msg: PointStamped):
        p = msg.point
        target = np.array([p.x, p.y, p.z], dtype=float)

        if not np.all(np.isfinite(target)):
            self.target_pos = None
            self.last_target_time = None
            
            self.publish_setpoint([0.0, 0.0, 0.0])
            return
        
        self.target_pos = target
        self.last_target_time = self.get_clock().now() # 목표점이 언제 들어왔는지 저장    


    def local_position_callback(self, msg: VehicleLocalPosition):
            self.drone_pos = np.array([msg.x, msg.y, msg.z], dtype=float)


    # def yaw_error_callback(self, msg: Float32):
    #         value = float(msg.data)
    
    #         if not math.isfinite(value):
    #             self.yaw_error_deg = math.nan
    #             self.last_yaw_error_time = None
    
    #             self.pid_yaw.reset()
    #             self.yaw_aligned_gate = False
    #             return
    
    #         # NaN 상태에서 정상값으로 복귀하는 순간에도 깨끗하게 시작
    #         if not math.isfinite(self.yaw_error_deg):
    #             self.pid_yaw.reset()
    
    #         self.yaw_error_deg = value
    #         self.last_yaw_error_time = self.get_clock().now() 


    def get_descent_vz(self, altitude_m: float) -> float:
        # PX4 VehicleLocalPosition.z 는 NED에서 'down'이 +값임(지상 쪽으로 증가) → 실제 고도는 -z 일 수 있음
        # 편하게 altitude_m 은 양수(지상으로부터 높이)로 계산했다고 가정
            if altitude_m > 20.0:
                return self.vz__20
            elif altitude_m > 10.0:
                return self.vz_20_10
            elif altitude_m > 5.0:
                return self.vz_10_5
            elif altitude_m > 3.0:
                return self.vz_5_3
            elif altitude_m > 0.0:
                return self.vz_3_0
            else:
                return 0.0


    def loop(self):
            
        if self.land_command_sent:
            return
        now = self.get_clock().now()
        # Offboard 준비 전에는 제어하지 않음
        if not self.is_offboard:
            return
        # dt 계산
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.prev_time = now

        if self.target_pos is None:
            self.publish_setpoint([0.0, 0.0, 0.0])
            return
        
        self.approach(dt)


    # def land_command(self,yaw_target_heading_deg: float,xy_err_now:float):
    #     now = self.get_clock().now()

    #     # 매 loop마다 속도 [0, 0, 0]을 계속 발행
    #     # Yaw는 목표 헤딩으로 계속 유지
    #     self.publish_setpoint([0.0, 0.0, 0.0])

        
    #     if self.pre_land_hold_start is None:
    #         self.pre_land_hold_start = now

    #         self.get_logger().info(
    #             f'Pre-LAND hold started: '
    #             f'publishing zero velocity [0, 0, 0] for '
    #             f'{self.pre_land_hold_time:.1f}s | '
    #         )
    #         return

    #     hold_elapsed = (
    #         now - self.pre_land_hold_start
    #     ).nanoseconds * 1e-9
        
    #     if hold_elapsed < self.pre_land_hold_time:
            
    #         return

        
    #     self.get_logger().info(
    #         f'Pre-LAND hold complete: '
    #         f'zero velocity maintained for {hold_elapsed:.2f}s. '
    #         f'Sending LAND command now.'
    #     )

    #     self.publish_vehicle_command(
    #         VehicleCommand.VEHICLE_CMD_NAV_LAND
    #     )
    #     self.land_command_sent = True

    #     self.get_logger().info(
    #         f'LAND command sent: '
    #         f'xy_err={xy_err_now:.3f}m, '                
    #     )


    def approach(self, dt:float):
    
        err_body = self.target_pos
        # err_yaw = self.yaw_error_deg

        xy_err = float(np.linalg.norm(err_body[:2]))
        alt = float(err_body[2])


        self.gate_check(xy_err, alt)

        if self.xy_alt_gate_one:
            vx_b = self.kp_xy_one * err_body[0]
            vy_b = self.kp_xy_one * err_body[1]
            vel_limit = self.vel_max_one


        elif self.xy_alt_gate_two:

            vx_b = self.kp_xy_two * err_body[0]
            vy_b = self.kp_xy_two * err_body[1]
            vel_limit = self.vel_max_two

        elif self.xy_alt_gate_final:

            vx_b = self.kp_xy_final * err_body[0]
            vy_b = self.kp_xy_final * err_body[1]
            vel_limit = self.vel_max_final

        vz_ned = self.get_descent_vz(alt) if alt > 2.0 else self.kp_z*(alt - self.gate_altitude)
        

        vxy = np.array([vx_b, vy_b])
        nrm = np.linalg.norm(vxy)
        if nrm > vel_limit:
            vxy *= (vel_limit / nrm)


        vel_body = np.array([vxy[0], vxy[1], 0.0])
        vel_ned_full = self.R_q @ vel_body if self.R_q_valid else vel_body
        vel_ned_xy = np.array([vel_ned_full[0], vel_ned_full[1], vz_ned])


        if self.z_aligned_gate and self.xy_aligned_gate:

            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.land_command_sent = True

            # xy_err_now = float(np.linalg.norm(self.target_pos[:2]))
            # self.land_command(0.0, xy_err_now)
            return

        self.publish_setpoint(vel_ned_xy)



    def gate_check(self, xy_err: float, alt: float):


        if alt <= self.gate_altitude:
            self.z_aligned_gate = True

        # if not math.isfinite(xy_err) or not math.isfinite(alt):
        #     self.xy_alt_gate_one = False
        #     self.xy_alt_gate_two = False
        #     self.xy_alt_gate_final = False

        #     self.xy_aligned_since = None
        #     self.xy_aligned_gate = False
        #     self.z_aligned_gate = False
        #     self.pre_land_hold_start = None
        #     return


        if alt >= 3.0:
            self.xy_alt_gate_one = True
            self.xy_alt_gate_two = False
            self.xy_alt_gate_final = False
            

        elif alt >= 2.0:
            self.xy_alt_gate_one = False
            self.xy_alt_gate_two = True
            self.xy_alt_gate_final = False
            

        else:
            self.xy_alt_gate_one = False
            self.xy_alt_gate_two = False
            self.xy_alt_gate_final = True
            

        xy_gate = xy_err <= self.xy_tol

        # XY 정렬이 완료되지 않았을 때만 검사
        if not self.xy_aligned_gate:
            xy_gate = xy_err <= self.xy_tol
            if xy_gate:
                self.xy_aligned_gate = True
            #     now = self.get_clock().now()

            #     if self.xy_aligned_since is None:
            #         self.xy_aligned_since = now

            #         self.get_logger().info(
            #             f"XY 정렬 hold 시작: error={xy_err:.3f}m"
            #         )

            #     hold_time = (
            #         now - self.xy_aligned_since
            #     ).nanoseconds * 1e-9

            #     if hold_time >= self.land_hold_time:
            #         self.xy_aligned_gate = True
            #         self.xy_aligned_since = None

            #         self.get_logger().info(
            #             f"XY 정렬 완료 및 고정: "
            #             f"error={xy_err:.3f}m, "
            #             f"hold={hold_time:.2f}s"
            #         )

            # else:
            #     # 정렬 완료 전 오차가 커지면 hold 시간 초기화
            #     self.xy_aligned_since = None

    def publish_setpoint(self, vel_vec):
        vel_ned = np.array(vel_vec, dtype=float).reshape(3)

        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [math.nan, math.nan, math.nan]
        sp.velocity = [
            float(vel_ned[0]),
            float(vel_ned[1]),
            float(vel_ned[2])
        ]
    
        sp.acceleration = [math.nan, math.nan, math.nan]
        sp.yaw = math.nan
        sp.yawspeed = math.nan

        self.setpoint_pub.publish(sp)
                

    def destroy_node(self):
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIDGuidanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        




