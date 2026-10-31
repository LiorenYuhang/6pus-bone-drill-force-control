"""Independent real-sensor/position-servo research entry; no J14 process."""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def start(context):
    root=Path(__file__).resolve().parents[1]
    cmd=[sys.executable,str(root/'scripts/research/force_position_control/runtime.py'),
         '--config',LaunchConfiguration('config').perform(context),
         '--duration',LaunchConfiguration('duration').perform(context)]
    mode=LaunchConfiguration('mode').perform(context)
    if mode not in ('read-only','active'): raise ValueError('mode must be read-only or active')
    cmd+=['--'+mode]
    if LaunchConfiguration('select_servo').perform(context).lower()=='true': cmd+=['--select-servo']
    if LaunchConfiguration('hold_current').perform(context).lower()=='true': cmd+=['--hold-current']
    log=LaunchConfiguration('log_dir').perform(context)
    if log: cmd+=['--log-dir',log]
    return [ExecuteProcess(cmd=cmd,output='screen')]


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    saved=root/'scripts/research/force_position_control/calibrated_config.json'
    default_config=saved if saved.exists() else root/'scripts/research/force_position_control/hardware_config.json'
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(default_config)),
        DeclareLaunchArgument('mode',default_value='read-only'),
        DeclareLaunchArgument('select_servo',default_value='false'),
        DeclareLaunchArgument('hold_current',default_value='false'),
        DeclareLaunchArgument('duration',default_value='0'),
        DeclareLaunchArgument('log_dir',default_value=''),
        OpaqueFunction(function=start)])
