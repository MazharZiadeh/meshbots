"""RViz with the meshbots view: merged maps, mesh links, swarm markers.

    ros2 launch meshbots rviz.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('meshbots')
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'meshbots.rviz')],
             parameters=[{'use_sim_time': True}],
             output='screen'),
    ])
