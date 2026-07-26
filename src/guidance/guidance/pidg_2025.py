#!/usr/bin/env python3

# 최종 수정 일자 
# 2026-05-08 00:01

# ================================================

import numpy as np
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray , Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Point
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleAttitude, VehicleLocalPosition, VehicleStatus, DistanceSensor

from mission_msgs.msg import ControlTick

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


class PID:
    def __init__(self, kp: float, ki: float, kd: float, lim: float, fc_d=15.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.lim = lim
        self.i = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.fc_d = fc_d

    def reset(self):
        self.i = 0.0
        self.prev_err = 0.0

    def update(self, err: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        d = (err - self.prev_err) / dt
        alpha = math.exp(-2.0 * math.pi * self.fc_d * dt)
        self.d_filt = alpha * self.d_filt + (1 - alpha) * d
        self.i += 0.5 * (err + self.prev_err) * dt
        u = self.kp * err + self.ki * self.i + self.kd * self.d_filt
        self.prev_err = err

        if u > self.lim:
            u = self.lim
        elif u < -self.lim:
            u = -self.lim
        return u


QOS_VEHICLE_DEFAULT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
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

        # PID parameters
        self.declare_parameter('kp', 0.27)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.01)
        self.declare_parameter('kp_z', 0.08)
        self.declare_parameter('ki_z', 0.00)
        self.declare_parameter('kd_z', 0.0)
        self.declare_parameter('vel_max', 0.8)
        self.declare_parameter('vel_max_z', 0.5)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        self.kp_z = self.get_parameter('kp_z').value
        self.ki_z = self.get_parameter('ki_z').value
        self.kd_z = self.get_parameter('kd_z').value


        # Tolerance for catching
        self.declare_parameter('xy_tol', 0.085)
        self.declare_parameter('z_tol', 0.025)
        self.xy_tol = self.get_parameter('xy_tol').value
        self.z_tol = self.get_parameter('z_tol').value

        # Descent speeds
        self.vel_max_xy = self.get_parameter('vel_max').value
        self.vel_max_z = self.get_parameter('vel_max_z').value
        self.declare_parameter('vz__20', 2.5)   # >10 m
        self.declare_parameter('vz_20_10', 1.0)   # >10 m
        self.declare_parameter('vz_10_5', 0.7)  # 10~5 m
        self.declare_parameter('vz_5_3', 0.35)   # 5~3 m
        self.declare_parameter('vz_3_0', 0.1)   # 3~0 m

        self.vz__20  = self.get_parameter('vz__20').value
        self.vz_20_10  = self.get_parameter('vz_20_10').value
        self.vz_10_5 = self.get_parameter('vz_10_5').value
        self.vz_5_3  = self.get_parameter('vz_5_3').value
        self.vz_3_0  = self.get_parameter('vz_3_0').value


        # State flags and metrics
        

        # PID controllers
        self.pid_x = PID(self.kp, self.ki, self.kd, self.vel_max_xy)
        self.pid_y = PID(self.kp, self.ki, self.kd, self.vel_max_xy)
        self.pid_z = PID(self.kp_z, self.ki_z, self.kd_z, self.vel_max_z)

        # Pose and target
        self.drone_pos = np.array([0.0, 0.0, 0.0])
        self.target_pos = None
        self.prev_time = self.get_clock().now()
        self.land_command_sent = False
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
        self.create_subscription(ControlTick, '/mission/control_tick', self.loop, QOS_VEHICLE_DEFAULT)
        # self.create_subscription(Float32MultiArray, '/detection/box_coords', self.area_callback, qos_profile)

        # Timers
        self.create_timer(0.1, self.offboard_timer_callback)
        # self.create_timer(0.02, self.loop)

        # Rotation matrix
        self.R_q = np.eye(3)
    
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

        if self.offboard_setpoint_counter_ < 11:
            zero_vec = np.zeros((3, 1))
            self.publish_setpoint(zero_vec, zero_vec)
            self.offboard_setpoint_counter_ += 1

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        for i in range(1, 8): setattr(msg, f'param{i}', params.get(f'param{i}', 0.0))
        msg.target_system = msg.target_component = msg.source_system = msg.source_component = 1
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

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.drone_pos = np.array([msg.x, msg.y, msg.z])

    def target_callback(self, msg: PointStamped):
        p = msg.point
        self.target_pos = np.array([p.x, p.y, p.z], dtype=float)

    # def area_callback(self, msg: Float32MultiArray):

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


    def loop(self,msg):
        if self.land_command_sent:
            return
        now = self.get_clock().now()
        # initial offboard skip
        if self.offboard_setpoint_counter_ <= 10:
            return

        # compute dt
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.prev_time = now

        if self.target_pos is None:
            zero_vec = np.zeros((3, 1))
            self.publish_setpoint(zero_vec, zero_vec)
            return

        pos_err = self.target_pos
        z_err = pos_err[2]
        
        xy_err = float(np.linalg.norm(pos_err[:2]))

        if z_err <= 1.4 and xy_err <= 0.1:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.land_command_sent = True
            return
        
        vx_b = self.pid_x.update(pos_err[0], dt)
        vy_b = self.pid_y.update(pos_err[1], dt)

        vxy = np.array([vx_b, vy_b])
        nrm = np.linalg.norm(vxy)
        if nrm > self.vel_max_xy:
            vxy *= (self.vel_max_xy / nrm)
        
        
        vel_vec_body = np.array([vxy[0], vxy[1], 0.0]).reshape((3, 1))
        
        vel_ned = self.R_q @ vel_vec_body

        vz_ned = self.get_descent_vz(z_err) if z_err > 1.4 else self.pid_z.update(z_err, dt)
        
        vel_ned[2,0] = vz_ned

        if z_err <= 1.6:
            vel_ned_vec = np.copy(vel_ned)  
            vel_ned_vec[2, 0] = 0.0         
        else:
            vel_ned_vec = vel_ned
        
        self.publish_setpoint(vel_vec_body, vel_ned_vec)
    

    def publish_setpoint(self, vel_vec, vel_ned_vec):
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [math.nan, math.nan, math.nan]
        sp.velocity = [float(vel_ned_vec[0, 0]), float(vel_ned_vec[1, 0]), float(vel_ned_vec[2, 0])]
        sp.acceleration = [math.nan, math.nan, math.nan]
        sp.yaw = math.nan
        sp.yawspeed = math.nan
        self.setpoint_pub.publish(sp)
        
        input_vel  = [vel_vec[0, 0], vel_vec[1, 0], vel_vec[2, 0]]
        output_vel = [vel_ned_vec[0, 0], vel_ned_vec[1, 0], vel_ned_vec[2, 0]]
        self.get_logger().info(
            f"Publishing velocity setpoints → body: {input_vel[0]:.3f}, {input_vel[1]:.3f}, {input_vel[2]:.3f},\n"
            f"NED: {output_vel[0]:.3f}, {output_vel[1]:.3f}, {output_vel[2]:.3f}"
        )
        

    def publish_setpoint(self, vel_vec,vel_ned_vec):
        
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [math.nan, math.nan, math.nan]
        sp.velocity = [float(vel_ned_vec[0]), float(vel_ned_vec[1]), float(vel_ned_vec[2])]
        sp.acceleration = [math.nan, math.nan, math.nan]
        sp.yaw = math.nan
        sp.yawspeed = math.nan
        self.setpoint_pub.publish(sp)
        input_vel  = [vel_vec[0][0], vel_vec[1][0], vel_vec[2][0]]                # body 프레임 속도
        output_vel = [vel_ned_vec[0][0], vel_ned_vec[1][0], vel_ned_vec[2][0]]    # NED 프레임 속도
        self.get_logger().info(
            f"Publishing velocity setpoints → body: {input_vel[0]:.3f},{input_vel[1]:.3f},{input_vel[2]:.3f},\n NED: {output_vel[0]:.3f},{output_vel[1]:.3f},{output_vel[2]:.3f}"
        )
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

