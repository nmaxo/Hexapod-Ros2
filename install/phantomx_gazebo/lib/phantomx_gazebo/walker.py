#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration

# Определение суставов по ногам
LEGS = {
    'lf': ['j_c1_lf', 'j_thigh_lf', 'j_tibia_lf'],
    'lm': ['j_c1_lm', 'j_thigh_lm', 'j_tibia_lm'],
    'lr': ['j_c1_lr', 'j_thigh_lr', 'j_tibia_lr'],
    'rf': ['j_c1_rf', 'j_thigh_rf', 'j_tibia_rf'],
    'rm': ['j_c1_rm', 'j_thigh_rm', 'j_tibia_rm'],
    'rr': ['j_c1_rr', 'j_thigh_rr', 'j_tibia_rr']
}

# Словарь начальных смещений для суставов c1
INITIAL_OFFSETS = {
    'j_c1_rf': 0,
    'j_c1_rr': 0,
    'j_c1_lf': 0,
    'j_c1_lr': 0,
    'j_c1_lm': 0,
    'j_c1_rm': 0,
}

# Группы триподов
GROUPS = {'A': ['lf', 'rm', 'lr'], 'B': ['rf', 'lm', 'rr']}

# Фаза переноса (шагающая нога)
TRANSFER_STEPS = [
    {'thigh': -0.3, 'tibia': 0.3, 'c1': 0.0},   # Подъем
    {'thigh': -0.3, 'tibia': 0.1, 'c1': -0.1},  # Вынос
    {'thigh': 0.0,  'tibia': 0.0, 'c1': -0.2},  # Опускание
]

# Фаза опоры (опорная нога — отталкивание назад)
STANCE_STEPS = [
    {'thigh': 0.0,  'tibia': 0.0, 'c1': 0.0},   # Нейтраль
    {'thigh': 0.0,  'tibia': 0.0, 'c1': 0.1},   # Начало отталкивания
    {'thigh': 0.05, 'tibia': 0.0, 'c1': 0.2},   # Сильнее назад
]

class TripodWalker(Node):
    def __init__(self):
        super().__init__('tripod_walker')
        # Публикация траекторий суставов
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        # Подписка на cmd_vel
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.group_cycle = ['A', 'B']
        self.current_group_idx = 0
        self.step_idx = 0
        self.timer = self.create_timer(0.3, self.publish_action)
        # Флаг для активации походки
        self.walk_enabled = False

    def cmd_vel_callback(self, msg: Twist):
        # Активируем походку, если есть ненулевая скорость
        self.walk_enabled = abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01
        self.get_logger().info(f'Walk enabled: {self.walk_enabled}, linear.x={msg.linear.x}')

    def publish_action(self):
        if not self.walk_enabled:
            return

        group = self.group_cycle[self.current_group_idx]
        traj = JointTrajectory()
        traj.joint_names = sum(LEGS.values(), [])
        pt = JointTrajectoryPoint()
        positions = []
        velocities = []

        for leg, joints in LEGS.items():
            # Определяем фазу: перенос или опора
            mode = 'transfer' if leg in GROUPS[group] else 'stance'
            config = TRANSFER_STEPS[self.step_idx] if mode == 'transfer' else STANCE_STEPS[self.step_idx]

            for joint in joints:
                # Определяем ключ по имени сустава
                key = 'c1' if 'c1' in joint else ('thigh' if 'thigh' in joint else 'tibia')
                val = config[key]

                # Инвертируем c1 для правых ног
                if key == 'c1' and leg in ['rf', 'rm', 'rr']:
                    val *= -1

                # Добавляем начальное смещение
                offset = INITIAL_OFFSETS.get(joint, 0.0)
                val += offset

                positions.append(val)
                # Рассчитываем скорость как изменение угла / время (0.5 секунды)
                velocity = abs(val) / 0.5
                velocities.append(velocity if val >= 0 else -velocity)

        pt.positions = positions
        pt.velocities = velocities
        pt.time_from_start = Duration(sec=0, nanosec=int(0.5 * 1e9))
        traj.points = [pt]
        self.pub.publish(traj)

        self.get_logger().info(f'Group {group} step {self.step_idx}')

        # Обновление шага и группы
        self.step_idx += 1
        if self.step_idx >= len(TRANSFER_STEPS):
            self.step_idx = 0
            self.current_group_idx = (self.current_group_idx + 1) % 2


def main(args=None):
    rclpy.init(args=args)
    node = TripodWalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
