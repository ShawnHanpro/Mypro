from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = os.path.expanduser('~/Mypro/create3_ws/src/edge_following')
    config_file = os.path.join(pkg_dir, 'yaml', 'config.yaml')
    return LaunchDescription([
        # 启动你的主节点
        Node(
            package='edge_following',
            executable='edge_following_node',
            name='edge_following',
            output='screen',
            parameters=[config_file]
        )
    ])
