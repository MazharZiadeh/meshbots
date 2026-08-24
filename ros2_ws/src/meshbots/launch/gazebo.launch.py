"""Gazebo bring-up: the arena world plus the /clock bridge.

    ros2 launch meshbots gazebo.launch.py [gui:=false] [world:=/path/to.sdf]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context, *args, **kwargs):
    share = get_package_share_directory('meshbots')
    world = LaunchConfiguration('world').perform(context) or \
        os.path.join(share, 'worlds', 'arena.sdf')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    gz_cmd = ['gz', 'sim', '-r', world]
    if not gui:
        gz_cmd = ['gz', 'sim', '-r', '-s', '--headless-rendering', world]

    return [
        ExecuteProcess(cmd=gz_cmd, output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=''),
        OpaqueFunction(function=setup),
    ])
