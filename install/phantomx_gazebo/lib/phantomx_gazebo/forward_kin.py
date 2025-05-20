#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class FKReaderLF(Node):
    def __init__(self):
        super().__init__('forward_kin')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Таймер на 1 секунду
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',   # базовый фрейм
                'tibia_lf',    # конечное звено ноги
                rclpy.time.Time()
            )
            t = trans.transform.translation
            r = trans.transform.rotation
            self.get_logger().info(
                f'Position tibia_lf relative to base_link: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}'
            )
            self.get_logger().info(
                f'Orientation (quaternion): x={r.x:.3f}, y={r.y:.3f}, z={r.z:.3f}, w={r.w:.3f}'
            )
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FKReaderLF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
