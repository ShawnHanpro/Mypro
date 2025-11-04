from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = os.path.expanduser('~/Mypro/create3_ws/src/spot_laser_collection')
    config_file = os.path.join(pkg_dir, 'yaml', 'config.yaml')
    return LaunchDescription([
        # 启动你的主节点
        Node(
            package='spot_laser_collection',
            executable='spot_laser_collection_node',
            name='spot_laser_collection',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_base_link_to_combined_scan',
            arguments=['0.07','0','0.18','0','0','0','base_link','combined_scan'],
            output='screen'
        )
    ])
