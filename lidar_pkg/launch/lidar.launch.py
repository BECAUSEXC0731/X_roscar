import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'lidar_pkg'

    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'lidar_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[config_path]
        ),

    ])