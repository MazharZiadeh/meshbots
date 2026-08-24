"""Top-level bring-up for the whole hive.

Composes (Husarion-tutorial style):
  gazebo.launch.py     — arena world + /clock bridge
  per robot            — model spawn + ros_gz bridge + the five-node stack,
                         everything under the robot's namespace, all tunables
                         from config/*.yaml
  radio_channel        — the RF physics simulator (the only global node)
  rviz.launch.py       — visualization

The team roster lives in config/team.yaml; the mission in
missions/delivery.yaml. Usage:

    ros2 launch meshbots sim.launch.py            # Gazebo GUI + RViz
    ros2 launch meshbots sim.launch.py gui:=false # headless sim + RViz
    ros2 launch meshbots sim.launch.py rviz:=false gui:=false
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_robot(share, robot):
    """Gazebo-facing actions for one robot: model + bridge."""
    name = robot['name']
    template = open(os.path.join(share, 'models', 'rover.sdf.template')).read()
    sdf_path = f'/tmp/meshbots_{name}.sdf'
    with open(sdf_path, 'w') as f:
        f.write(template.replace('{NAME}', name)
                        .replace('{COLOR}', robot['color']))
    return [
        Node(package='ros_gz_sim', executable='create',
             name=f'spawn_{name}',
             arguments=['-file', sdf_path, '-name', name,
                        '-x', str(robot['x']), '-y', str(robot['y']),
                        '-z', '0.12', '-Y', str(robot['yaw'])],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             namespace=name, name='bridge',
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
             parameters=[{'use_sim_time': True}],
             output='screen'),
    ]


def robot_stack(share, robot, mission):
    """The five per-robot nodes, namespaced, configured from config/*.yaml."""
    name = robot['name']
    cfg = lambda f: os.path.join(share, 'config', f)  # noqa: E731
    targets_flat = [float(v) for xy in mission['targets'] for v in xy]
    base = [float(v) for v in mission['base']]
    sim_time = {'use_sim_time': True}

    stacks = [
        ('mesh_radio', cfg('mesh.yaml'), {}),
        ('localizer', cfg('localization.yaml'),
         {'spawn': [robot['x'], robot['y'], robot['yaw']],
          'anchors': targets_flat}),
        ('mapper', cfg('mapping.yaml'), {}),
        ('swarm', cfg('swarm.yaml'),
         {'targets': targets_flat, 'base': base}),
        ('navigator', cfg('navigation.yaml'), {}),
    ]
    return [
        Node(package='meshbots', executable=exe,
             namespace=name, name=exe,
             parameters=[cfg_file, {**overrides, **sim_time}],
             output='screen')
        for exe, cfg_file, overrides in stacks
    ]


def setup(context, *args, **kwargs):
    share = get_package_share_directory('meshbots')
    team = yaml.safe_load(open(os.path.join(share, 'config', 'team.yaml')))
    mission = yaml.safe_load(
        open(os.path.join(share, 'missions', 'delivery.yaml')))
    robots = team['robots']
    targets_flat = [float(v) for xy in mission['targets'] for v in xy]

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())]

    for robot in robots:
        actions += spawn_robot(share, robot)
        actions += robot_stack(share, robot, mission)

    spawns_flat = [v for r in robots for v in (r['x'], r['y'], r['yaw'])]
    actions.append(Node(
        package='meshbots', executable='radio_channel', name='radio_channel',
        parameters=[{'robots': [r['name'] for r in robots],
                     'spawns': spawns_flat,
                     'anchors': targets_flat,
                     'use_sim_time': True}],
        output='screen'))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'rviz.launch.py')),
        condition=IfCondition(LaunchConfiguration('rviz'))))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
