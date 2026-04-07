import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_swarm_core = get_package_share_directory('swarm_core')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    xacro_file = os.path.join(pkg_swarm_core, 'urdf', 'drone.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    nodes = [gz_sim]
    fleet = [('alpha', 0.0), ('bravo', 2.0), ('charlie', -2.0)]

    for name, y_pos in fleet:
        nodes.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=name, parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ))
        
        nodes.append(Node(
            package='ros_gz_sim', executable='create',
            arguments=['-topic', f'/{name}/robot_description', '-name', name, '-z', '0.5', '-y', str(y_pos)]
        ))
        
        # THE FIX: Absolute Gazebo paths, remapped to clean ROS namespaces
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            namespace=name,
            arguments=[
                f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
            ],
            remappings=[
                (f'/model/{name}/odometry', 'odom'),
                (f'/model/{name}/cmd_vel', 'cmd_vel')
            ]
        ))

    return LaunchDescription(nodes)
