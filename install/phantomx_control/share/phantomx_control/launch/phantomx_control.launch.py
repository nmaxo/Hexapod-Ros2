import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Загрузка параметров из YAML файла
        Node(
            package='ros2param',
            executable='param',
            name='phantomx_param_loader',
            output='screen',
            arguments=['load', os.path.join(
                get_package_share_directory('phantomx_control'), 'config', 'phantomx_control.yaml')],
        ),
        
        # Запуск контроллеров через controller_manager
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            arguments=[
                'j_c1_lf_position_controller', 'j_c1_rf_position_controller', 'j_c1_lm_position_controller',
                'j_c1_rm_position_controller', 'j_c1_lr_position_controller', 'j_c1_rr_position_controller',
                'j_thigh_lf_position_controller', 'j_thigh_rf_position_controller', 'j_thigh_lm_position_controller',
                'j_thigh_rm_position_controller', 'j_thigh_lr_position_controller', 'j_thigh_rr_position_controller',
                'j_tibia_lf_position_controller', 'j_tibia_rf_position_controller', 'j_tibia_lm_position_controller',
                'j_tibia_rm_position_controller', 'j_tibia_lr_position_controller', 'j_tibia_rr_position_controller',
                'joint_state_controller'
            ]
        ),
        
        # Конвертирование состояний суставов в TF для rviz
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            remappings=[('/joint_states', '/phantomx/joint_states')]
        )
    ])
