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
    
    # 1. 获取 URDF 文件路径
    urdf_file_name = 'lidar.urdf'
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file_name)

    # 读取 URDF 内容
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    # 2. 获取 RViz 配置文件路径
    rviz_config_dir = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'lidar.rviz')

    return LaunchDescription([
        # 节点 1: Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # 节点 2: Lidar Driver Node
        Node(
            package=pkg_name,
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[config_path]
        ),

        # 节点 3: RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir] 
        ),
    ])