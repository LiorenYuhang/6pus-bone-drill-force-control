"""6PUS Active Attitude Stabilization Launch File.

Launches:
1. Base H30 IMU Node (/pus6/imu_base/data)
2. Platform H30 IMU Node (/pus6/imu_platform/data)
3. 6PUS Read-Only Motor Feedback Node (/pus6/joint_feedback/timed)
4. Force Active Stabilization Controller Node (Roll=P, Pitch/Yaw=PD, Mode 3 Dry-Run)
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('pus6_force_control'))
    default_config = str(pkg_share / 'config' / 'active_stabilization.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config,
            description='Path to active stabilization config YAML',
        ),
        DeclareLaunchArgument(
            'auto_enable',
            default_value='true',
            description='Whether to automatically enable attitude stabilization PID',
        ),
        DeclareLaunchArgument(
            'launch_imus',
            default_value='true',
            description='Whether to launch dual H30 IMU nodes (base and platform)',
        ),
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            description='Whether to launch read-only pus6_motor_driver_node',
        ),
        DeclareLaunchArgument(
            'base_imu_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00',
            description='Serial port for Base IMU',
        ),
        DeclareLaunchArgument(
            'plat_imu_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00',
            description='Serial port for Platform IMU',
        ),
        DeclareLaunchArgument(
            'motor_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Serial port for 6PUS motors UART',
        ),
        DeclareLaunchArgument(
            'hardware_mode',
            default_value='read_only',
            description='Hardware execution mode: read_only (strictly 0 writes enforced)',
        ),

        # 1. Base H30 IMU Node -> /pus6/imu_base/data
        Node(
            package='h30_imu',
            executable='h30_imu_node',
            namespace='pus6/imu_base',
            name='h30_imu',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('base_imu_port'),
                'baud_rate': 460800,
                'frame_id': 'pus6_base_imu_link',
                'world_frame': 'world',
                'imu_topic': '/pus6/imu_base/data',
                'publish_euler': True,
            }],
            condition=IfCondition(LaunchConfiguration('launch_imus')),
        ),

        # 2. Platform H30 IMU Node -> /pus6/imu_platform/data
        Node(
            package='h30_imu',
            executable='h30_imu_node',
            namespace='pus6/imu_platform',
            name='h30_imu',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('plat_imu_port'),
                'baud_rate': 460800,
                'frame_id': 'pus6_platform_imu_link',
                'world_frame': 'world',
                'imu_topic': '/pus6/imu_platform/data',
                'publish_euler': True,
            }],
            condition=IfCondition(LaunchConfiguration('launch_imus')),
        ),

        # 3. Read-Only 6PUS Motor Feedback Node -> /pus6/joint_feedback/timed
        Node(
            package='pus6_motor_driver',
            executable='motor_driver_node',
            name='pus6_motor_driver_node',
            output='screen',
            parameters=[{
                'driver_architecture': 'legacy_single_process',
                'hardware_mode': LaunchConfiguration('hardware_mode'),
                'port': LaunchConfiguration('motor_port'),
                'baudrate': 921600,
                'poll_rate_hz': 50.0,
                'auto_connect': True,
                'auto_configure': False,
                'use_j7_servo': False,
                'write_rate_hz': 0.0,
            }],
            condition=IfCondition(LaunchConfiguration('launch_driver')),
        ),

        # 4. Force Active Stabilization Controller Node
        Node(
            package='pus6_force_control',
            executable='force_active_stabilization',
            name='force_active_stabilization',
            parameters=[
                LaunchConfiguration('params_file'),
                {'auto_enable': LaunchConfiguration('auto_enable')},
            ],
            output='screen',
        ),
    ])
