"""Bring up the whole hive: Gazebo arena, three rovers, per-robot node stacks
(mesh radio, mapper, swarm, navigator), the RF channel simulator and RViz.

    ros2 launch meshbots sim.launch.py            # with Gazebo GUI + RViz
    ros2 launch meshbots sim.launch.py gui:=false # headless sim + RViz
    ros2 launch meshbots sim.launch.py rviz:=false gui:=false
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOTS = [
    # name,      spawn x, y, yaw,   chassis color
    ('rover_1', -11.0, -11.0, 0.785, '0.85 0.20 0.15'),
    ('rover_2', -12.8, -10.2, 0.785, '0.15 0.50 0.90'),
    ('rover_3', -10.2, -12.8, 0.785, '0.95 0.75 0.10'),
]


def setup(context, *args, **kwargs):
    share = get_package_share_directory('meshbots')
    world = os.path.join(share, 'worlds', 'arena.sdf')
    template = open(os.path.join(share, 'models', 'rover.sdf.template')).read()
    mission = yaml.safe_load(
        open(os.path.join(share, 'missions', 'delivery.yaml')))
    targets_flat = [float(v) for xy in mission['targets'] for v in xy]
    base = [float(v) for v in mission['base']]
    use_sim_time = {'use_sim_time': True}

    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    actions = []

    gz_cmd = ['gz', 'sim', '-r', world]
    if not gui:
        gz_cmd = ['gz', 'sim', '-r', '-s', '--headless-rendering', world]
    actions.append(ExecuteProcess(cmd=gz_cmd, output='screen'))

    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'))

    spawns_flat = []
    for name, sx, sy, syaw, color in ROBOTS:
        spawns_flat += [sx, sy, syaw]
        sdf_path = f'/tmp/meshbots_{name}.sdf'
        with open(sdf_path, 'w') as f:
            f.write(template.replace('{NAME}', name).replace('{COLOR}', color))

        actions.append(Node(
            package='ros_gz_sim', executable='create',
            name=f'spawn_{name}',
            arguments=['-file', sdf_path, '-name', name,
                       '-x', str(sx), '-y', str(sy), '-z', '0.12',
                       '-Y', str(syaw)],
            output='screen'))

        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'bridge_{name}',
            arguments=[
                f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/model/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            remappings=[
                (f'/model/{name}/cmd_vel', f'/{name}/cmd_vel'),
                (f'/model/{name}/odometry', f'/{name}/odom'),
                (f'/model/{name}/scan', f'/{name}/scan'),
            ],
            parameters=[use_sim_time],
            output='screen'))

        for pkg_exec, extra in [
            ('mesh_radio', {}),
            ('localizer', {'spawn': [sx, sy, syaw],
                           'anchors': targets_flat,
                           'eval_dir': '/tmp/meshbots_eval'}),
            ('mapper', {}),
            ('swarm', {'targets': targets_flat, 'base': base}),
            ('navigator', {}),
        ]:
            actions.append(Node(
                package='meshbots', executable=pkg_exec,
                name=f'{pkg_exec}_{name}',
                parameters=[{'robot': name, **extra, **use_sim_time}],
                output='screen'))

    actions.append(Node(
        package='meshbots', executable='radio_channel',
        name='radio_channel',
        parameters=[{'robots': [r[0] for r in ROBOTS],
                     'spawns': spawns_flat,
                     'anchors': targets_flat,
                     **use_sim_time}],
        output='screen'))

    actions.append(Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(share, 'config', 'mesh.rviz')],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
