#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TestNode(Node):
    def __init__(self):
        super().__init__('sub_node')
        self.sub = self.create_subscription(
            Float32, 
            '/sensor/value',
            self.subscribe_msg,
            10)
        self.sub
        self.get_logger().info("Тестовая нода запущена!")

    def subscribe_msg(self, msg):
        self.get_logger().info(f" сообщение: {msg.data}")

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