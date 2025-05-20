       
from hexapod.Ax18 import Ax18
from rclpy.node import Node
import rclpy
from threading import Lock
from collections import defaultdict
from custom_interfaces.msg import MotorCmd

class DxlDriver(Node):
    def __init__(self):
        super().__init__('dxl_driver')
        self.motor_positions = {}  # motor_id -> position
        self.lock = Lock()
        self.timer_period = 0.05  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        Ax18.DEVICENAME = '/dev/ttyUSB0'
        Ax18.BAUDRATE = 500000
        Ax18.connect()

        self.create_subscription(MotorCmd, '/dxl_com', self.command_callback, 10)

        self.create_publisher()

    def command_callback(self, msg):
        with self.lock:
            self.motor_positions[id for id in msg.motor_id.split()]
            self.get_logger().info(self.motor_positions)


def main(args=None):
    rclpy.init(args=args)
    node = DxlDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__  == "__main()__":
    main()