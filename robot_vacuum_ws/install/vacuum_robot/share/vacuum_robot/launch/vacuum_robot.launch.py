import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    package_name = 'vacuum_robot'
    
    # 1. 获取路径
    pkg_path = get_package_share_directory(package_name)
    # # 假设你的 aws 房屋模型在这个包里或者在已安装路径
    # world_file = os.path.join(get_package_share_directory('vacuum_robot'), 'worlds', 'small_house.world')

    # 你的路径（请根据实际情况修改）
    aws_house_pkg = "/home/shan2/Mypro/gazebo_ws/aws-robomaker-small-house-world"
    
    # 【关键步骤】设置环境变量，让 Gazebo 能找到墙壁、沙发等模型
    # 假设模型都在该包的 'models' 文件夹下
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(aws_house_pkg, 'models')
    )

    # 你的 world 文件路径
    world_path = os.path.join(aws_house_pkg, 'worlds', 'small_house.world')

    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    # 2. 解析 Xacro
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml()}

    # 3. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )

    # 4. 发布机器人状态 (Robot State Publisher)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 5. 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'vacuum_robot', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])