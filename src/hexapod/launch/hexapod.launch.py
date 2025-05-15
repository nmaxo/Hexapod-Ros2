# hexapod.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    # Запуск координатора
    coordinator_node = Node(
        package='hexapod',
        executable='coordinator',
        name='coordinator'
    )
    ld.add_action(coordinator_node)

    fsr_test_node = Node(
        package='hexapod',
        executable='fsr_test_node',
        name='fsr_data'
    )
    ld.add_action(fsr_test_node)
    


    # Запуск 6 контроллеров ног
    leg_1_controller_node = Node(
            package='hexapod',
            executable='leg_controller',
            name=f'leg_1_controller',
            parameters=[{'leg_num': 1}]
        )
    ld.add_action(leg_1_controller_node)
    
    leg_2_controller_node = Node(
            package='hexapod',
            executable='leg_controller',
            name=f'leg_2_controller',
            parameters=[{'leg_num': 2}]
        )
    ld.add_action(leg_2_controller_node)

    fsr_test_node  = Node(
        package='hexapod',
        executable= 'fsr_test_node'
    )
    ld.add_action(fsr_test_node)
    
    
    return ld