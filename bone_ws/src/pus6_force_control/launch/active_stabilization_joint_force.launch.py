"""Controller only: no UART/IMU device is opened by this launch file."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config=str(Path(get_package_share_directory('pus6_force_control'))/'config'/'active_stabilization_joint_force.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=config),
        DeclareLaunchArgument('auto_enable',default_value='true'),
        Node(package='pus6_force_control',executable='force_active_stabilization',
             name='force_active_stabilization',
             parameters=[LaunchConfiguration('params_file'),
                         {'auto_enable': LaunchConfiguration('auto_enable')}],
             output='screen')
    ])
