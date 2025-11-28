from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    return LaunchDescription([
        # 启动你的主节点
        Node(
            package='edge_following',
            executable='dwa',
            name='dwa',
            output='screen'
        )
    ])
