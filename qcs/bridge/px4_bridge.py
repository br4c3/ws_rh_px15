#!/usr/bin/env python3

import json
import math
import os
import queue
import sys
import threading
import time

BRIDGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("ROS_LOG_DIR", os.path.join(BRIDGE_ROOT, ".ros-log"))
os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)

import rclpy
from mavros_msgs.msg import RCOut
from px4_msgs.msg import (
    ActuatorMotors,
    AirspeedValidated,
    BatteryStatus,
    PositionSetpointTriplet,
    EstimatorStatusFlags,
    SensorGps,
    SensorCombined,
    VehicleCommand,
    VehicleGlobalPosition,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleLocalPosition,
    VehicleLocalPositionSetpoint,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

NAVIGATION_MODES = {
    VehicleStatus.NAVIGATION_STATE_MANUAL: "MANUAL",
    VehicleStatus.NAVIGATION_STATE_ALTCTL: "ALTITUDE",
    VehicleStatus.NAVIGATION_STATE_POSCTL: "POSITION",
    VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: "MISSION",
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: "HOLD",
    VehicleStatus.NAVIGATION_STATE_AUTO_RTL: "RETURN",
    VehicleStatus.NAVIGATION_STATE_ACRO: "ACRO",
    VehicleStatus.NAVIGATION_STATE_OFFBOARD: "OFFBOARD",
    VehicleStatus.NAVIGATION_STATE_STAB: "STABILIZED",
    VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: "TAKEOFF",
    VehicleStatus.NAVIGATION_STATE_AUTO_LAND: "LAND",
}

MODE_NAV_STATES = {
    mode: nav_state
    for nav_state, mode in NAVIGATION_MODES.items()
}


def finite_or(value, fallback=0.0):
    return float(value) if math.isfinite(value) else fallback


class Px4ElectronBridge(Node):
    def __init__(self):
        super().__init__("arecada_electron_bridge")

        self.last_px4_message = 0.0
        self.last_position_setpoint_message = 0.0
        self.last_actuator_motors_message = 0.0
        self.commands = queue.Queue()
        self.state = {
            "flightMode": "WAITING FOR PX4",
            "armed": False,
            "failsafe": False,
            "vtolState": "MC",
            "gpsSatellites": 0,
            "gpsFix": 0,
            "hdop": 0.0,
            "gpsPosition": {
                "latitude": None,
                "longitude": None,
                "altitude": None,
                "horizontalAccuracy": None,
                "verticalAccuracy": None,
            },
            "altitude": 0.0,
            "airspeed": 0.0,
            "groundSpeed": 0.0,
            "throttle": 0.0,
            "battery": 0.0,
            "heading": 0.0,
            "latitude": None,
            "longitude": None,
            "missionWaypoints": [],
            "sensor": {
                "accelerometer": [0.0, 0.0, 0.0],
                "gyroscope": [0.0, 0.0, 0.0],
                "accelClipping": 0,
                "gyroClipping": 0,
            },
            "estimate": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "localPosition": [0.0, 0.0, 0.0],
                "velocity": [0.0, 0.0, 0.0],
                "horizontalError": 0.0,
                "verticalError": 0.0,
                "globalPosition": [None, None, None],
                "referencePosition": [None, None, None],
                "deadReckoning": False,
                "tiltAligned": False,
                "yawAligned": False,
                "gnssFusion": False,
                "barometerFusion": False,
                "inertialFault": False,
            },
            "setpoint": {
                "localPosition": [0.0, 0.0, 0.0],
                "velocity": [0.0, 0.0, 0.0],
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "valid": False,
            },
            "sourceTimestamps": {},
            "simulation": {
                "trayPosition": [
                    # Gazebo ENU (X=east, Y=north) -> PX4 NED (X=north, Y=east)
                    float(os.environ.get("TRAY_Y", "5.0")),
                    float(os.environ.get("TRAY_X", "0.0")),
                    float(os.environ.get("TRAY_Z", "0.04")),
                ],
            },
        }

        self.subscribe_compatible(
            VehicleStatus,
            ["/fmu/out/vehicle_status_v4", "/fmu/out/vehicle_status"],
            self.on_vehicle_status,
        )
        self.subscribe_compatible(
            VehicleLocalPosition,
            [
                "/fmu/out/vehicle_local_position_v1",
                "/fmu/out/vehicle_local_position",
            ],
            self.on_local_position,
        )
        self.subscribe_compatible(
            AirspeedValidated,
            ["/fmu/out/airspeed_validated_v1", "/fmu/out/airspeed_validated"],
            self.on_airspeed,
        )
        self.create_subscription(
            ActuatorMotors,
            "/fmu/out/actuator_motors",
            self.on_actuator_motors,
            PX4_QOS,
        )
        self.create_subscription(
            RCOut,
            "/mavros/rc/out",
            self.on_mavros_rc_out,
            10,
        )
        self.create_subscription(
            SensorGps,
            "/fmu/out/vehicle_gps_position",
            self.on_gps,
            PX4_QOS,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.on_global_position,
            PX4_QOS,
        )
        self.create_subscription(
            PositionSetpointTriplet,
            "/fmu/out/position_setpoint_triplet",
            self.on_mission_setpoints,
            PX4_QOS,
        )
        self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self.on_sensor_combined,
            PX4_QOS,
        )
        self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.on_vehicle_attitude,
            PX4_QOS,
        )
        self.subscribe_compatible(
            VehicleLocalPositionSetpoint,
            [
                "/fmu/out/vehicle_local_position_setpoint",
                "/fmu/out/vehicle_local_position_setpoint_v1",
            ],
            self.on_local_position_setpoint,
        )
        self.create_subscription(
            VehicleAttitudeSetpoint,
            "/fmu/out/vehicle_attitude_setpoint",
            self.on_attitude_setpoint,
            PX4_QOS,
        )
        self.create_subscription(
            EstimatorStatusFlags,
            "/fmu/out/estimator_status_flags",
            self.on_estimator_status,
            PX4_QOS,
        )
        self.subscribe_compatible(
            BatteryStatus,
            ["/fmu/out/battery_status_v1", "/fmu/out/battery_status"],
            self.on_battery,
        )

        self.command_publisher = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            PX4_QOS,
        )

        self.create_timer(0.2, self.publish_state)
        self.create_timer(0.05, self.process_commands)

    def subscribe_compatible(self, message_type, topics, callback):
        for topic in topics:
            self.create_subscription(
                message_type,
                topic,
                callback,
                PX4_QOS,
            )

    def mark_px4_active(self):
        self.last_px4_message = time.monotonic()

    def on_vehicle_status(self, message):
        self.mark_px4_active()
        self.state["flightMode"] = NAVIGATION_MODES.get(
            int(message.nav_state),
            f"MODE {message.nav_state}",
        )
        self.state["armed"] = (
            message.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        self.state["failsafe"] = bool(message.failsafe)

        if message.in_transition_mode:
            self.state["vtolState"] = "TRANSITION"
        elif message.vehicle_type == VehicleStatus.VEHICLE_TYPE_FIXED_WING:
            self.state["vtolState"] = "FW"
        else:
            self.state["vtolState"] = "MC"

    def on_local_position(self, message):
        self.mark_px4_active()
        self.state["sourceTimestamps"]["vehicleLocalPosition"] = int(
            message.timestamp
        )

        if message.z_valid:
            self.state["altitude"] = max(0.0, -finite_or(message.z))

        heading_degrees = math.degrees(finite_or(message.heading))
        self.state["heading"] = heading_degrees % 360
        self.state["estimate"]["localPosition"] = [
            finite_or(message.x),
            finite_or(message.y),
            finite_or(message.z),
        ]
        self.state["estimate"]["velocity"] = [
            finite_or(message.vx),
            finite_or(message.vy),
            finite_or(message.vz),
        ]
        self.state["groundSpeed"] = math.hypot(
            finite_or(message.vx),
            finite_or(message.vy),
        )
        if message.xy_global:
            self.state["estimate"]["referencePosition"] = [
                float(message.ref_lat),
                float(message.ref_lon),
                float(message.ref_alt) if message.z_global else None,
            ]

    def on_airspeed(self, message):
        self.mark_px4_active()
        self.state["sourceTimestamps"]["airspeedValidated"] = int(
            message.timestamp
        )
        self.state["airspeed"] = max(
            0.0,
            finite_or(message.calibrated_airspeed_m_s),
        )

    def on_actuator_motors(self, message):
        self.mark_px4_active()
        self.last_actuator_motors_message = time.monotonic()
        self.state["sourceTimestamps"]["actuatorMotors"] = int(
            message.timestamp
        )
        active_controls = [
            float(value)
            for value in message.control
            if math.isfinite(value) and value >= 0.0
        ]
        self.state["throttle"] = (
            sum(active_controls) / len(active_controls) * 100.0
            if active_controls
            else 0.0
        )

    def on_mavros_rc_out(self, message):
        if time.monotonic() - self.last_actuator_motors_message < 1.0:
            return

        motor_outputs = [
            float(value)
            for value in list(message.channels)[:4]
            if value > 0
        ]
        if not motor_outputs:
            self.state["throttle"] = 0.0
            return

        self.state["throttle"] = max(
            0.0,
            min(100.0, sum(motor_outputs) / len(motor_outputs) / 10.0),
        )

    def on_gps(self, message):
        self.mark_px4_active()
        self.state["sourceTimestamps"]["sensorGps"] = int(message.timestamp)
        self.state["gpsSatellites"] = int(message.satellites_used)
        self.state["gpsFix"] = int(message.fix_type)
        self.state["hdop"] = finite_or(message.hdop)
        self.state["gpsPosition"] = {
            "latitude": float(message.latitude_deg),
            "longitude": float(message.longitude_deg),
            "altitude": float(message.altitude_msl_m),
            "horizontalAccuracy": finite_or(message.eph),
            "verticalAccuracy": finite_or(message.epv),
        }

    def on_battery(self, message):
        self.mark_px4_active()
        self.state["battery"] = max(
            0.0,
            min(100.0, finite_or(message.remaining) * 100),
        )

    def on_global_position(self, message):
        self.mark_px4_active()
        self.state["sourceTimestamps"]["vehicleGlobalPosition"] = int(
            message.timestamp
        )

        if message.lat_lon_valid:
            self.state["latitude"] = finite_or(message.lat)
            self.state["longitude"] = finite_or(message.lon)
            self.state["estimate"]["globalPosition"] = [
                float(message.lat),
                float(message.lon),
                float(message.alt) if message.alt_valid else None,
            ]

        self.state["estimate"]["horizontalError"] = finite_or(message.eph)
        self.state["estimate"]["verticalError"] = finite_or(message.epv)
        self.state["estimate"]["deadReckoning"] = bool(message.dead_reckoning)

    def on_sensor_combined(self, message):
        self.mark_px4_active()
        self.state["sourceTimestamps"]["sensorCombined"] = int(
            message.timestamp
        )
        self.state["sensor"]["accelerometer"] = [
            finite_or(value)
            for value in message.accelerometer_m_s2
        ]
        self.state["sensor"]["gyroscope"] = [
            finite_or(value)
            for value in message.gyro_rad
        ]
        self.state["sensor"]["accelClipping"] = int(
            message.accelerometer_clipping
        )
        self.state["sensor"]["gyroClipping"] = int(message.gyro_clipping)

    def on_vehicle_attitude(self, message):
        self.mark_px4_active()
        self.state["sourceTimestamps"]["vehicleAttitude"] = int(
            message.timestamp
        )
        roll, pitch, yaw = self.quaternion_to_euler(message.q)
        self.state["estimate"]["roll"] = roll
        self.state["estimate"]["pitch"] = pitch
        self.state["estimate"]["yaw"] = yaw

    def quaternion_to_euler(self, quaternion):
        w, x, y, z = [finite_or(value) for value in quaternion]
        sin_roll = 2 * (w * x + y * z)
        cos_roll = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = max(-1.0, min(1.0, 2 * (w * y - z * x)))
        pitch = math.asin(sin_pitch)
        sin_yaw = 2 * (w * z + x * y)
        cos_yaw = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(sin_yaw, cos_yaw)
        return (
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw) % 360,
        )

    def on_local_position_setpoint(self, message):
        self.mark_px4_active()
        self.last_position_setpoint_message = time.monotonic()
        self.state["sourceTimestamps"]["vehicleLocalPositionSetpoint"] = int(
            message.timestamp
        )
        setpoint = self.state["setpoint"]
        setpoint["localPosition"] = [
            finite_or(message.x),
            finite_or(message.y),
            finite_or(message.z),
        ]
        setpoint["velocity"] = [
            finite_or(message.vx),
            finite_or(message.vy),
            finite_or(message.vz),
        ]
        setpoint["yaw"] = math.degrees(finite_or(message.yaw)) % 360
        setpoint["valid"] = any(
            math.isfinite(value)
            for value in [message.x, message.y, message.z]
        )

    def on_attitude_setpoint(self, message):
        self.mark_px4_active()
        self.state["sourceTimestamps"]["vehicleAttitudeSetpoint"] = int(
            message.timestamp
        )
        roll, pitch, yaw = self.quaternion_to_euler(message.q_d)
        setpoint = self.state["setpoint"]
        setpoint["roll"] = roll
        setpoint["pitch"] = pitch
        setpoint["yaw"] = yaw

    def on_estimator_status(self, message):
        self.mark_px4_active()
        self.state["sourceTimestamps"]["estimatorStatusFlags"] = int(
            message.timestamp
        )
        estimate = self.state["estimate"]
        estimate["tiltAligned"] = bool(message.cs_tilt_align)
        estimate["yawAligned"] = bool(message.cs_yaw_align)
        estimate["gnssFusion"] = bool(message.cs_gnss_pos)
        estimate["barometerFusion"] = bool(message.cs_baro_hgt)
        estimate["deadReckoning"] = bool(
            message.cs_inertial_dead_reckoning
        )
        estimate["inertialFault"] = any(
            [
                message.fs_bad_acc_vertical,
                message.fs_bad_acc_clipping,
                message.fs_bad_hdg,
            ]
        )

    def on_mission_setpoints(self, message):
        self.mark_px4_active()
        waypoints = []

        for label, setpoint in [
            ("PREVIOUS", message.previous),
            ("CURRENT", message.current),
            ("NEXT", message.next),
        ]:
            if not setpoint.valid:
                continue

            waypoints.append(
                {
                    "label": label,
                    "latitude": finite_or(setpoint.lat),
                    "longitude": finite_or(setpoint.lon),
                    "altitude": finite_or(setpoint.alt),
                    "type": int(setpoint.type),
                }
            )

        self.state["missionWaypoints"] = waypoints

    def publish_state(self):
        connected = time.monotonic() - self.last_px4_message < 2.0
        self.state["setpoint"]["valid"] = (
            self.state["setpoint"]["valid"]
            and time.monotonic() - self.last_position_setpoint_message < 1.0
        )
        payload = {
            "type": "telemetry",
            "connected": connected,
            **self.state,
        }
        print(json.dumps(payload, separators=(",", ":")), flush=True)

    def process_commands(self):
        while not self.commands.empty():
            command = self.commands.get_nowait()
            command_type = command.get("type")

            if command_type == "flightMode":
                nav_state = MODE_NAV_STATES.get(command.get("mode"))
                if nav_state is not None:
                    self.send_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_SET_NAV_STATE,
                        param1=float(nav_state),
                    )

            elif command_type == "arm":
                self.send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0 if command.get("armed") else 0.0,
                )

            elif command_type == "vtolState":
                target_state = command.get("state")
                mav_vtol_state = 4.0 if target_state == "FW" else 3.0
                self.send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION,
                    param1=mav_vtol_state,
                )

    def send_vehicle_command(self, command, **parameters):
        message = VehicleCommand()
        message.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        message.command = command
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True

        for name, value in parameters.items():
            setattr(message, name, value)

        self.command_publisher.publish(message)


def read_commands(node):
    for line in sys.stdin:
        try:
            node.commands.put(json.loads(line))
        except json.JSONDecodeError:
            continue


def main():
    rclpy.init(args=None)
    node = Px4ElectronBridge()

    input_thread = threading.Thread(
        target=read_commands,
        args=(node,),
        daemon=True,
    )
    input_thread.start()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
