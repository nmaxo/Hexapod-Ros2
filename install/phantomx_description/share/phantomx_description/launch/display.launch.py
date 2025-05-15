from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Путь к xacro-файлу робота
    urdf_file = PathJoinSubstitution([
        FindPackageShare('phantomx_description'), 'urdf', 'phantomx.urdf'
    ])

    # Параметр для выбора запуска GUI joint_state_publisher
    use_jsp_gui = LaunchConfiguration('use_jsp_gui', default='true')

    # Параметр robot_description, который развернёт xacro в URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_jsp_gui',
            default_value='true',
            description='Запускать ли joint_state_publisher_gui'
        ),

        # Запуск robot_state_publisher с параметром robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Запуск joint_state_publisher_gui, если use_jsp_gui=true
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(use_jsp_gui)
        ),

        # Запуск обычного joint_state_publisher, если use_jsp_gui=false
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(use_jsp_gui)
        ),

        # Запуск RViz с конфигурационным файлом (если есть)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('phantomx_description'), 'rviz', 'robot.rviz'
            ])]
        )
    ])
