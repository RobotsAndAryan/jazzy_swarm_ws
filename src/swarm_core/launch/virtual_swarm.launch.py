from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_core',
            executable='supervisor_node',
            name='flight_supervisor',
            namespace='alpha_actual',
            output='screen',
        ),
        Node(
            package='swarm_core',
            executable='supervisor_node',
            name='flight_supervisor',
            namespace='bravo_actual',
            output='screen',
        ),
        Node(
            package='swarm_core',
            executable='supervisor_node',
            name='flight_supervisor',
            namespace='charlie_actual',
            output='screen',
        )
    ])
