import rclpy
from rclpy.node import Node
from custom_interfaces.msg import LegCommand, LegStatus
from std_msgs.msg import Float32
from .ciclograms import air_phase, stance_phase

class One_leg_coordinator(Node):
    def __init__(self):
        super().__init__('one_leg_coordinator')
        self.cmd_pub = self.create_publisher(LegCommand, '/leg_command', 10)
        self.sub = self.create_subscription(
            Float32, 
            '/sensor/value',
            self.subscribe_msg,
            10)
        self.status_sub = self.create_subscription(
            LegStatus, 
            '/leg_status', 
            self.status_callback, 10)
        
        self.leg_status = {4:'idle'} 
        self.step_count = 0
    
    def subscribe_msg(self, msg):
        pass
        #self.get_logger().info(f" сообщение: {msg.data}")
        
    def status_callback(self, msg):
        self.leg_status[msg.leg_num] = msg.state
        self.get_logger().info(f" Статус: {msg.state}")
        
        # Проверяем статус конкретной ноги (например, ноги №4)
        if msg.leg_num == 4 and msg.state == 'idle':
            self.make_step()
    
    def make_step(self):
        if self.step_count % 2 == 0:
            self.send_command(4, 'start_transfer', air_phase)
        else:
            self.send_command(4, "start_support", stance_phase)
        self.step_count += 1
        self.get_logger().info(f'текущий шаг : {self.step_count}')# Не забываем увеличивать счётчик!

    def send_command(self, leg_num, command_type, trajectory):
        msg = LegCommand()
        msg.leg_num = leg_num  # Указываем номер ноги
        msg.command_type = command_type
        msg.trajectory = trajectory
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Отправлено сообщение: {msg}")

def main():
    rclpy.init()
    node = One_leg_coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()