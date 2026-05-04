import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('askar_simulation')
    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg, 'worlds', 'teknofest_v2.world')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Gazebo başlat
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        # Robotu Gazebo'ya spawn et
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'askar_robot',
                      '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),
    ])
