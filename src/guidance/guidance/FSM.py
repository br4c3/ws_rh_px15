#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from enum import Enum, auto

from std_msgs.msg import Empty
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleStatus,
)
from mission_msgs.msg import ControlTick

# Vehicle_command topic QOS 맞추기 위해서 임의로 하나 만듬
QOS_VEHICLE_DEFAULT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)

# QoS 프로파일 정의
QOS_DEFAULT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)



class FSM(Node):
    """
    드론의 미션을 관리하는 FSM 노드.
    QGC 미션 종료(LOITER)를 감지한 후, 제어권을 받아 Offboard 미션을 수행합니다.
    이 노드는 Setpoint를 생성하지 않고, 상태 관리 및 모드 변경만 담당합니다.
    """
    def __init__(self):
        super().__init__("fsm")

        # ---------- 파라미터 선언 및 초기화 ----------
        self.declare_parameter("rate_hz", 20.0)


        self.rate_hz = self.get_parameter("rate_hz").value
        self.dt_nom = 1.0 / max(self.rate_hz, 1.0)
        
        # ---------- ROS Pub/Sub ----------
        self.tick_pub = self.create_publisher(ControlTick, "/mission/control_tick", QOS_VEHICLE_DEFAULT)
        self.prev_time = self.get_clock().now()
        # ---------- 타이머 ----------
        self.timer = self.create_timer(self.dt_nom, self._publish_tick)
        self.get_logger().info(f"✅ [FSM] started ({self.rate_hz} Hz)")

    def _publish_tick(self):
        msg = ControlTick()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.dt = float('nan')
        self.tick_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        if node.is_armed:
            node.get_logger().info("Node is shutting down, sending DISARM command.")
            node._send_vehicle_command(PX4.VEHICLE_CMD_COMPONENT_ARM_DISARM, PX4.DISARM_COMMAND)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
