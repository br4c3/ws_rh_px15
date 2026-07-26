import rclpy, sys, termios, tty, select
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand

MAV_CMD_DO_SET_ACTUATOR = 187
SYS_ID  = 1      # PX4 기본 system_id
COMP_ID = 1

class ServoKeyNode(Node):
    def __init__(self):
        super().__init__('servo_key_mavcmd')

        pub_qos = rclpy.qos.QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        self.servo1_val = 0.0   # Actuator Set-3
        self.servo2_val = -0.76   # Actuator Set-4
        self.servo3_val = 0.0   # Actuator Set-5

        # 50 Hz 주기
        self.create_timer(0.02, self._servo_timer_cb)

    # ──────────────────────────────────────
    def _servo_timer_cb(self):
        self._process_key_input()
        self._publish_vehicle_command()

    # ──────────────────────────────────────
    def _publish_vehicle_command(self):
        now = self.get_clock().now().nanoseconds // 1000

        msg = VehicleCommand()
        msg.timestamp          = now
        

        # param1~param6 → Actuator Set 1~6 값
        msg.param1 = msg.param2 = float('nan')
        msg.param3 = float(self.servo1_val)     # Set-3 상하 
        msg.param4 = float(self.servo2_val)     # Set-4 좌우 
        msg.param5 = float(self.servo3_val)     # Set-5 그립
        msg.param6 = float('nan')
        msg.param7 = 0.0

        msg.command            = MAV_CMD_DO_SET_ACTUATOR
        msg.target_system      = SYS_ID
        msg.target_component   = COMP_ID
        msg.source_system      = SYS_ID
        msg.source_component   = COMP_ID
        msg.from_external      = True           # Companion Computer → PX4

        self.cmd_pub.publish(msg)
        self.get_logger().info(
            f"[MAV_CMD 187] set3={self.servo1_val:+.2f}  "
            f"set4={self.servo2_val:+.2f}  set5={self.servo3_val:+.2f}")

    # ──────────────────────────────────────
    def _process_key_input(self):
        key = self._get_key()
        if key == 'q':
            self.servo1_val = +1.0
        elif key == 'e':
            self.servo1_val = -1.0
        elif key == 'w':
            self.servo1_val = 0.0
        elif key == 'a':
            self.servo2_val = -1.0 #1350
        elif key == 'd':
            self.servo2_val = -0.44 #1420
        elif key == 's':
            self.servo2_val = -0.76 #1380 
        elif key == 'z':
            self.servo3_val = max(-1.0, self.servo3_val - 0.05)
        elif key == 'c':
            self.servo3_val = min(+1.0, self.servo3_val + 0.05)
        elif key == 'x':
            self.servo3_val = 0.0
        elif key == 'b':
            self.servo3_val = 0.43
        elif key == '\x1b':
            self.get_logger().info('종료합니다.')
            rclpy.shutdown()
            return
# 1380 disarm 
# 1420 max 1600 
# 1350 min

    # ──────────────────────────────────────
    @staticmethod
    def _get_key(timeout: float = 0.02) -> str:
        fd = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            r, _, _ = select.select([sys.stdin], [], [], timeout)
            return sys.stdin.read(1) if r else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)

# ──────────────────────────────────────────
def main():
    rclpy.init()
    node = ServoKeyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
