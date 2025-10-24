from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数文件路径
    share_dir = get_package_share_directory('mpu6050driver')
    default_params_file = os.path.join(share_dir, 'params', 'mpu6050.yaml')

    # 声明 Launch 参数
    params_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the ROS2 parameters file to use.'
    )

    # MPU6050 节点
    mpu6050_node = Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file]
    )

    # RViz2 节点
    rviz_config_file = os.path.join(share_dir, 'rviz', 'mpu6050.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        params_declare,
        mpu6050_node,
        rviz_node
    ])
