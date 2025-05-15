import rclpy
from rclpy.node import Node
from custom_interfaces.msg import LegCommand, LegStatus
from std_msgs.msg import Float32
from .ciclograms import air_phase, stance_phase
import time

class Coordinator(Node):
    def __init__(self):
        time.sleep(3.0)
        super().__init__('coordinator')
        self.cmd_pub = self.create_publisher(LegCommand, '/leg_command', 10)
        self.status_sub = self.create_subscription(
            LegStatus, 
            '/leg_status', 
            self.status_callback, 
            10)
        
        # Инициализация статусов для всех 6 ног
        self.leg_status = {i: 'idle' for i in range(1, 7)}
        self.step_count = 0
        
        self.groups = [[1], [2]]
        self.current_group_idx = 0
        self.waiting_for_transfer = False
        
        # Запуск первого шага
        self.activate_group(self.current_group_idx)

    def status_callback(self, msg):
        self.leg_status[msg.leg_num] = msg.state
        
        if self.waiting_for_transfer:
            current_group = self.groups[self.current_group_idx]
            if all(self.leg_status[leg] == 'idle' for leg in current_group):
                self.get_logger().info(f"Группа {current_group} завершила перенос")
                # Переключаем группу и активируем следующую
                self.current_group_idx = (self.current_group_idx + 1) % 2
                self.activate_group(self.current_group_idx)
                self.step_count += 1

    def activate_group(self, group_idx):
        time.sleep(1.0)
        """Активирует перенос для указанной группы и опору для противоположной"""
        transfer_group = self.groups[group_idx]
        support_group = self.groups[1 - group_idx]
        
        # Отправляем команды на перенос
        for leg in transfer_group:
            self.send_command(leg, 'start_transfer', air_phase)
        
        # Отправляем команды на опору
        for leg in support_group:
            self.send_command(leg, 'start_support', stance_phase)
        
        self.waiting_for_transfer = True
        self.get_logger().info(f"Активирована группа {transfer_group}")

    def send_command(self, leg_num, command_type, trajectory):
        msg = LegCommand()
        msg.leg_num = leg_num
        msg.command_type = command_type
        msg.trajectory = trajectory
        self.cmd_pub.publish(msg)
        self.get_logger().debug(f"Команда для ноги {leg_num}: {command_type}")

def main():
    rclpy.init()
    node = Coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()