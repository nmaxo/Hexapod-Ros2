#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import LegStatus

class TestNode(Node):
    def __init__(self):
        super().__init__('hexapod_test_node')
        self.pub = self.create_publisher(LegStatus, '/test_status', 10)
        self.timer = self.create_timer(1.0, self.publish_msg)
        self.counter = 0
        self.get_logger().info("Тестовая нода запущена!")

    def publish_msg(self):
        msg = LegStatus()
        msg.leg_num = 1
        msg.state = f"state_{self.counter}"
        msg.sensor_value = float(self.counter)
        self.pub.publish(msg)
        self.get_logger().info(f"Отправлено сообщение: {msg}")
        self.counter += 1

def main():
    rclpy.init()
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()