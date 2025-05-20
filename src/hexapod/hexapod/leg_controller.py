#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import LegCommand, LegStatus, SensorData,MotorCmd
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from hexapod.Ax18 import Ax18
import time

class OneLegController(Node):
    def __init__(self):
        super().__init__('leg_controller')
        # Параметр ноги теперь обязательный
        self.declare_parameter('leg_num', rclpy.Parameter.Type.INTEGER)
        self.leg_num = self.get_parameter('leg_num').get_parameter_value().integer_value
        self.get_logger().info(f"Инициализация ноги {self.leg_num}")

        self.state = "idle"
        self.sensor_value = 0
        self.sensor_threshold = 50  # Пороговое значение датчика
        self.trajectory = []
        self.traj_idx = 0
        self.current_point = Point()
        self.last_sensor_time = self.get_clock().now()
        
        # Инициализация моторов

        base_id = (self.leg_num - 1) * 3
        self.motor_coxa = base_id + 1
        self.motor_femur = base_id + 2
        self.motor_tibia = base_id + 3


        # ROS 2 интерфейсы
        self.status_pub = self.create_publisher(LegStatus, f'/leg_status', 10)

        self.command_sub = self.create_subscription(
            LegCommand, f'/leg_command', self.command_callback, 10)
        
        self.dxl_pub = self.create_publisher(MotorCmd,f'/dxl_com',10)

        self.timer_dxl = self.create_timer(1, self.dxl_publisher)
        
        # Подписка на сенсоры с фильтрацией по номеру ноги
        self.sensor_sub = self.create_subscription(
            SensorData, '/legs_data', self.sensor_callback, 10)
            
        # Более частый таймер для плавного управления
        self.timer = self.create_timer(1, self.fsm_step)



    def command_callback(self, msg):
        """Обработка команд управления"""
        if msg.leg_num != self.leg_num and msg.leg_num != 0:  
            return
            
        self.get_logger().info(f"Получена команда: {msg.command_type}")
        
        if msg.command_type in ["start_support", "start_transfer"]:
            self.handle_trajectory_command(msg)
        elif msg.command_type == "reset":
            self.state = "idle"
            self.stop_motors()
        elif msg.command_type == "set_sensor_threshold":
            self.sensor_threshold = msg.value

    def dxl_publisher(self):
        msg = MotorCmd()
        msg.servo_ids = [self.motor_coxa, self.motor_femur, self.motor_tibia]
        
        # Явное преобразование Point в список
        msg.positions = [
            int(self.current_point.x),
            int(self.current_point.y), 
            int(self.current_point.z)
        ]
        
        self.dxl_pub.publish(msg)
        self.get_logger().info(f"Моторам {msg.servo_ids} отправлены позиции {msg.positions} ")
    

    def handle_trajectory_command(self, msg):
        """Обработка команд с траекторией"""
        if not msg.trajectory:
            self.get_logger().error("Получена пустая траектория!")
            return
            
        self.trajectory = msg.trajectory
        self.traj_idx = 0
        self.state = "transfer" if "transfer" in msg.command_type else "support"

    def sensor_callback(self, msg):
        """Обработка данных с датчика с проверкой номера ноги"""
        self.sensor_value = msg.legs[self.leg_num-1]
        self.get_logger().debug(f"Датчик ноги {self.leg_num}: {msg.legs[self.leg_num-1]}")

    def fsm_step(self):
        """Конечный автомат управления ногой"""
        status_msg = LegStatus()
        status_msg.leg_num = self.leg_num
        status_msg.state = self.state
        self.status_pub.publish(status_msg)

        # # Проверка актуальности данных сенсора
        # if (self.get_clock().now() - self.last_sensor_time).nanoseconds > 1e9:  # 1 секунда
        #     self.get_logger().warn("Нет свежих данных с датчика!")
        #     self.state = "idle"
        #     return

        if self.state == "idle":
            pass
            
        elif self.state == "support":
            self.handle_support_phase()
            
        elif self.state in ["transfer", "touching"]:
            self.handle_transfer_phase()

    def handle_support_phase(self):
        """Фаза опоры"""
        if self.traj_idx < len(self.trajectory):
            self.execute_trajectory_step()
            self.traj_idx += 1
        else:
            self.state = "idle"

    def handle_transfer_phase(self):
        """Фаза переноса и поиска опоры"""
        if self.sensor_value > self.sensor_threshold:
            self.state = "support"
            return
            
        if self.traj_idx < len(self.trajectory):
            self.execute_trajectory_step()
            self.traj_idx += 1
        else:
            self.state = "touching"
            self.search_support()

    def execute_trajectory_step(self):
        """Выполнение одного шага траектории"""
        try:
            self.current_point = self.trajectory[self.traj_idx]
            
 
        except Exception as e:
            self.get_logger().error(f"Ошибка выполнения шага: {str(e)}")
            self.state = "idle"

    def search_support(self):
        """Поиск опоры при отсутствии контакта"""
        self.get_logger().info("Поиск опоры...")
        
        # Текущие позиции
        
        # Медленно опускаем ногу
        # for i in range(10):
        #     if self.sensor_value > self.sensor_threshold:
        #         break
                
        #     new_pos = current_pos[2] - 5  # Опускаем tibia
        #     self.motor_tibia.set_goal_position(new_pos)
        #     time.sleep(1)


    def shutdown(self):
        Ax18.disconnect()
        self.get_logger().info(f"Нога {self.leg_num} отключена")

def main():
    rclpy.init()
    node = OneLegController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()