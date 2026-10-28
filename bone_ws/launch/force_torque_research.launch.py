"""Source-tree ROS 2 launch entry for synthetic/replay research only."""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    entry=root/'scripts/research/force_torque_control/research_entry.py'
    config=root/'scripts/research/force_torque_control/example_mock_config.json'
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(config)),
        DeclareLaunchArgument('mode',default_value='ACTUATOR_FORCE_CONTROL'),
        DeclareLaunchArgument('input',default_value='synthetic'),
        DeclareLaunchArgument('duration',default_value='0'),
        DeclareLaunchArgument('status_source',default_value='mock'),
        DeclareLaunchArgument('log_dir',default_value=''),
        ExecuteProcess(cmd=[sys.executable,str(entry),'--config',LaunchConfiguration('config'),
                            '--mode',LaunchConfiguration('mode'),'--input',LaunchConfiguration('input'),
                            '--status-source',LaunchConfiguration('status_source'),
                            '--duration',LaunchConfiguration('duration'),'--log-dir',LaunchConfiguration('log_dir')],output='screen')])
