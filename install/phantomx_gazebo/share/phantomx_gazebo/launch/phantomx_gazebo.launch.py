import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Пакеты
    pkg_gazebo   = get_package_share_directory('phantomx_gazebo')
    pkg_desc     = get_package_share_directory('phantomx_description')
    pkg_control  = get_package_share_directory('phantomx_control')

    # Файлы
    urdf_file       = os.path.join(pkg_desc,    'urdf', 'phantomx.urdf')
    world_file      = os.path.join(pkg_gazebo,  'worlds', 'empty.world')
    controllers_yaml= os.path.join(pkg_control, 'config', 'phantomx_control.yaml')

    # Читаем URDF
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Контроллеры
    controllers = [
        'joint_state_broadcaster',
        'joint_trajectory_controller'
    ]

    # ros2_control node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': True},
            controllers_yaml
        ],
    )

    # Спавнеры контроллеров
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawn_{c}',
            output='screen',
            arguments=[c]
        )
        for c in controllers
    ]

    # Задержка для запуска спавнеров
    delayed_spawners = TimerAction(
        period=5.0,
        actions=controller_spawners
    )

    # Запуск Gazebo server с нужными аргументами
    gzserver = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # Запуск Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    walker_node = Node(
    package='phantomx_gazebo',
    executable='walker',
    name='walker'
    )

    # Спавн модели в Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'phantomx', '-file', urdf_file, '-z', '1.5', '-x', '0.3', '-y', '0.2']
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher_node,
        spawn_entity_node,
        controller_manager_node,
        delayed_spawners,
        walker_node
    ])
