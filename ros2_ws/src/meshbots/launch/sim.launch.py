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
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_rosbot(robot):
    """Spawn a Husarion ROSbot (built from source) instead of the built-in
    rover: their namespace-aware spawn_robot.yaml brings up the model,
    ros2_control stack, EKF and laser filter under the robot's namespace."""
    from ament_index_python.packages import get_package_share_directory as gps
    name = robot['name']
    return [IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(gps('rosbot_gazebo'), 'launch', 'spawn_robot.yaml')),
        launch_arguments={
            'namespace': name,
            'robot_model': 'rosbot',
            'rviz': 'False',
            'x': str(robot['x']), 'y': str(robot['y']),
            'z': '0.1', 'yaw': str(robot['yaw']),
        }.items())]


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


def robot_stack(share, robot, mission, seed, eval_dir, chassis, formation):
    """The five per-robot nodes, namespaced, configured from config/*.yaml."""
    name = robot['name']
    cfg = lambda f: os.path.join(share, 'config', f)  # noqa: E731
    targets_flat = [float(v) for xy in mission['targets'] for v in xy]
    base = [float(v) for v in mission['base']]
    sim_time = {'use_sim_time': True}

    # The ROSbot chassis differs from the built-in rover in three ways:
    # odometry comes from its EKF, the usable lidar topic is the filtered
    # one, and its ros2_control stack wants TwistStamped commands.
    rosbot = chassis == 'rosbot'
    odom_remap = [('odom', 'odometry/filtered')] if rosbot else []
    scan_remap = [('scan', 'scan_filtered')] if rosbot else []
    nav_over = {'cmd_vel_stamped': True} if rosbot else {}

    stacks = [
        ('mesh_radio', cfg('mesh.yaml'), {'eval_dir': eval_dir}, []),
        ('localizer', cfg('localization.yaml'),
         {'spawn': [robot['x'], robot['y'], robot['yaw']],
          'anchors': targets_flat,
          'noise_seed': seed, 'eval_dir': eval_dir}, odom_remap),
        ('mapper', cfg('mapping.yaml'), {'eval_dir': eval_dir}, scan_remap),
        ('swarm', cfg('swarm.yaml'),
         {'targets': targets_flat, 'base': base,
          'eval_dir': eval_dir}, []),
        ('navigator', cfg('navigation.yaml'), nav_over, scan_remap),
    ]
    planner_cfg = {'informative': 'formation.yaml',
                   'aware': 'formation_aware.yaml'}.get(formation)
    if planner_cfg:
        stacks.append(('formation_planner', cfg(planner_cfg),
                       {'anchors': targets_flat}, []))
    return [
        Node(package='meshbots', executable=exe,
             namespace=name, name=exe,
             parameters=[cfg_file, {**overrides, **sim_time}],
             remappings=remaps,
             output='screen')
        for exe, cfg_file, overrides, remaps in stacks
    ]


def setup(context, *args, **kwargs):
    share = get_package_share_directory('meshbots')
    team = yaml.safe_load(open(os.path.join(share, 'config', 'team.yaml')))
    mission = yaml.safe_load(
        open(os.path.join(share, 'missions', 'delivery.yaml')))
    robots = team['robots']
    targets_flat = [float(v) for xy in mission['targets'] for v in xy]
    seed = int(LaunchConfiguration('seed').perform(context))
    eval_dir = LaunchConfiguration('eval_dir').perform(context)
    chassis = LaunchConfiguration('chassis').perform(context)
    formation = LaunchConfiguration('formation').perform(context)

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'),
                          'world': LaunchConfiguration('world')}.items())]

    for robot in robots:
        if chassis == 'rosbot':
            actions += spawn_rosbot(robot)
        else:
            actions += spawn_robot(share, robot)
        actions += robot_stack(share, robot, mission, seed, eval_dir,
                               chassis, formation)

    spawns_flat = [v for r in robots for v in (r['x'], r['y'], r['yaw'])]
    actions.append(Node(
        package='meshbots', executable='radio_channel', name='radio_channel',
        parameters=[{'robots': [r['name'] for r in robots],
                     'spawns': spawns_flat,
                     'anchors': targets_flat,
                     'odom_topic': ('odometry/filtered'
                                    if chassis == 'rosbot' else 'odom'),
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
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('chassis', default_value='builtin',
                              choices=['builtin', 'rosbot']),
        DeclareLaunchArgument('formation', default_value='fixed',
                              choices=['fixed', 'informative', 'aware']),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('eval_dir', default_value='/tmp/meshbots_eval'),
        OpaqueFunction(function=setup),
    ])
