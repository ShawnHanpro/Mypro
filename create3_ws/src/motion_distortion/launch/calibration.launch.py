from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动你的主节点
        Node(
            package='show_laser',
            executable='calibration',
            name='calibration_node',
            output='screen',
            parameters=[{
                'target_frame': 'odom'
            }]
        ),

        # 静态 TF： odom -> base_laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_world_to_base_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar'],
            output='screen'
        )
    ])
