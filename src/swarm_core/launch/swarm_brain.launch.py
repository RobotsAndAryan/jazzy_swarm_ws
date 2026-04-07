from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []
    fleet = ['alpha', 'bravo', 'charlie']

    for name in fleet:
        nodes.append(Node(
            package='swarm_core',
            executable='boid_controller',
            namespace=name,
            parameters=[{'drone_name': name}],
            output='screen'
        ))

    return LaunchDescription(nodes)
