import rclpy
from rclpy.node import Node
from custom_interfaces.msg import LegCommand, LegStatus,SensorData
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from hexapod.Ax18 import Ax18
from hexapod.ciclograms import *
import time

class OneLegController(Node):
    def __init__(self):
        super().__init__('leg_controller')
        self.leg_num = self.declare_parameter('leg_num', 1)
        self.leg_num = self.get_parameter('leg_num').value()
        self.get_logger().info(f"Инициализация ноги {self.leg_num}")

        self.state = "idle"
        self.sensor_value = 0
        self.trajectory = []
        self.traj_idx = 0
        
        # Инициализация моторов
        Ax18.DEVICENAME = '/dev/ttyUSB0'
        Ax18.BAUDRATE = 500000
        Ax18.connect()
        base_id = (self.leg_num - 1) * 3
        self.motor_coxa = Ax18(base_id + 1)
        self.motor_femur = Ax18(base_id + 2)
        self.motor_tibia = Ax18(base_id + 3)
        for motor in [self.motor_coxa, self.motor_femur, self.motor_tibia]:
            motor.set_torque_enable(200)
            motor.set_moving_speed(100)

        # ROS 2 интерфейсы
        self.status_pub = self.create_publisher(LegStatus, '/leg_status', 10)
        self.command_sub = self.create_subscription(
            LegCommand, '/leg_command', self.command_callback, 10)
        self.sensor_sub = self.create_subscription(
            SensorData, '/sensor/value', self.sensor_callback, 10)
        self.timer = self.create_timer(1.0, self.fsm_step)

    def command_callback(self, msg):
        self.get_logger().info(f"Получена команда: {msg.command_type}")
        if msg.command_type in ["start_support", "start_transfer"]:
            if not msg.trajectory:
                self.get_logger().error("Получена пустая траектория!")
                return
                
            self.trajectory = msg.trajectory
            self.traj_idx = 0
            self.state = "transfer" if "transfer" in msg.command_type else "support"
        elif msg.command_type == "reset":
            self.state = "idle"

    def sensor_callback(self, msg):
        if msg.leg_num ==self.leg_num:
            self.sensor_value = msg.value
            self.get_logger().info(f"Показание с датчика {msg.value}")

    def fsm_step(self):
        status_msg = LegStatus()
        status_msg.leg_num = self.leg_num
        status_msg.state = self.state
        self.status_pub.publish(status_msg)

        if self.state == "idle":
            pass
        elif self.state == "support":
            if self.traj_idx < len(self.trajectory):
                self.execute_trajectory_step()
                self.traj_idx += 1
            else:
                self.state = "idle"
        elif self.state == "transfer":
            # if self.sensor_value > 250:
            #     self.state = "stopped"
            #     self.stop_motors()
            if self.traj_idx < len(self.trajectory):
                self.execute_trajectory_step()
                self.traj_idx += 1
            else:
                if self.is_support_found():
                    self.state = "support"
                else:
                    self.state = "touching"
        elif self.state == "touching":
            self.touching_step()
            if self.is_support_found():
                self.state = "support"
        elif self.state == "stopped":
            pass

    def execute_trajectory_step(self):
        try:
            point = self.trajectory[self.traj_idx]
            self.get_logger().info(f"Выполнение точки: {point.x}, {point.y}, {point.z}")
            
            # Управление моторами
            self.motor_coxa.set_goal_position(int(point.x))
            self.motor_femur.set_goal_position(int(point.y))
            self.motor_tibia.set_goal_position(int(point.z))
        
        except Exception as e:
            self.get_logger().error(f"Ошибка выполнения шага: {str(e)}")
            self.state = "idle"

    def touching_step(self):
        print("Начинаю нащупывание")
        # current_pos = self.motor_coxa.get_present_position()
        # new_pos = current_pos - 10
        # self.motor_coxa.set_goal_position(new_pos)
        # time.sleep(0.1)

    def is_support_found(self):
        return 50 <= self.sensor_value


    def shutdown(self):
        pass
        # for motor in [self.motor_coxa, self.motor_femur, self.motor_tibia]:
        #     motor.set_torque_enable(0)
        # Ax18.disconnect()

def main():
    rclpy.init()
    node = OneLegController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()