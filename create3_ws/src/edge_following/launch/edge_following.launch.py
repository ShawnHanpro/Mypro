from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # 查找的是install目录下的yaml文件，需要编译才会更新
    # pkg_share = get_package_share_directory('edge_following')

    # 查找绝对路径下的文件，无需编译，但移植程序需要修改此处文件位置
    pkg_dir = os.path.expanduser('~/Mypro/create3_ws/src/edge_following')

    config_file = os.path.join(pkg_dir, 'yaml', 'config.yaml')

    # 使用ekf计算odom
    ekf_config = os.path.join(pkg_dir, 'yaml', 'ekf.yaml')

    return LaunchDescription([
        # 启动你的主节点
        Node(
            package='edge_following',
            executable='edge_following_node',
            name='edge_following',
            output='screen',
            parameters=[config_file]
        ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[ekf_config],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='imu_to_base_link_filter',
        #     arguments=[
        #         '0.051', '0.035', '0.069',   # x, y, z
        #         '0', '0', '0', '1',          # quaternion x, y, z, w
        #         'base_link_filtered',           # parent frame
        #         'imu'                    # child frame
        #     ]
        # )
    ])
