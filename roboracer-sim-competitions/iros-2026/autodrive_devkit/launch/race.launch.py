"""Race stack: AutoDRIVE bridge + Nav2 AMCL + controller.

Defaults are the IROS map (`track-20260731`) and its raceline.
Default controller is pure pursuit (`pp_backup`). Swap back to MPC with one arg:

  ros2 launch autodrive_roboracer race.launch.py
  ros2 launch autodrive_roboracer race.launch.py controller:=mpc

Override map/raceline at launch:

  ros2 launch autodrive_roboracer race.launch.py \\
      map:=/path/to/map.yaml \\
      waypoint_file:=/path/to/waypoints.csv \\
      map_name:=track-20260731
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _first_file(*paths):
    for p in paths:
        if p and os.path.isfile(p):
            return os.path.abspath(p)
    return os.path.abspath(paths[0])


def _pkg_paths():
    pkg = get_package_share_directory('autodrive_roboracer')
    src = '/home/autodrive_devkit/src/autodrive_devkit'
    here = os.path.dirname(__file__)
    return pkg, src, here


def _controller_node(context, *args, **kwargs):
    """Start pp_backup (default) or mpc. Both stay registered; one arg swaps."""
    controller = LaunchConfiguration('controller').perform(context).strip()
    waypoint_file = LaunchConfiguration('waypoint_file')
    pkg, src, here = _pkg_paths()

    if controller == 'mpc':
        params_file = _first_file(
            os.path.join(pkg, 'config', 'mpc_params.yaml'),
            os.path.join(src, 'mpc_params.yaml'),
            os.path.join(here, '..', 'mpc_params.yaml'),
        )
        return [
            LogInfo(msg=f'Race controller: kinematic MPC ({params_file})'),
            Node(
                package='autodrive_roboracer',
                executable='mpc_node',
                name='mpc_node',
                emulate_tty=True,
                output='screen',
                parameters=[
                    params_file,
                    {'waypoint_file': waypoint_file},
                ],
            ),
        ]

    params_file = _first_file(
        os.path.join(pkg, 'config', 'pp_backup_params.yaml'),
        os.path.join(src, 'pp_backup_params.yaml'),
        os.path.join(here, '..', 'pp_backup_params.yaml'),
    )
    return [
        LogInfo(msg=f'Race controller: pure pursuit backup ({params_file})'),
        Node(
            package='autodrive_roboracer',
            executable='pp_backup_node',
            name='pp_backup_node',
            emulate_tty=True,
            output='screen',
            parameters=[
                params_file,
                {'waypoint_file': waypoint_file},
            ],
        ),
    ]


def generate_launch_description():
    pkg, src, here = _pkg_paths()

    default_map = _first_file(
        os.path.join(pkg, 'maps', 'track-20260731', 'map.yaml'),
        os.path.join(src, 'maps', 'track-20260731', 'map.yaml'),
        os.path.join(here, '..', 'maps', 'track-20260731', 'map.yaml'),
        os.path.join(pkg, 'maps', 'practice_track.yaml'),
        os.path.join(src, 'practice_track.yaml'),
    )
    default_wps = _first_file(
        os.path.join(pkg, 'racelines', 'track-20260731-line', 'waypoints.csv'),
        os.path.join(src, 'racelines', 'track-20260731-line', 'waypoints.csv'),
        os.path.join(here, '..', 'racelines', 'track-20260731-line',
                     'waypoints.csv'),
        os.path.join(pkg, 'practice_waypoints.csv'),
        os.path.join(src, 'practice_waypoints.csv'),
    )

    map_yaml = LaunchConfiguration('map')
    map_name = LaunchConfiguration('map_name')

    localize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'localize.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='PLAYGROUND_DATA_DIR', value=pkg),
        DeclareLaunchArgument(
            'controller', default_value='pp_backup',
            description='Race controller: pp_backup (default) or mpc'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Occupancy map YAML (track-20260731 by default)'),
        DeclareLaunchArgument(
            'waypoint_file', default_value=default_wps,
            description='Raceline CSV s,x,y,theta,velocity'),
        DeclareLaunchArgument(
            'map_name', default_value='track-20260731',
            description='Playground map slug under maps/ (for store.map_meta)'),

        Node(
            package='autodrive_roboracer',
            executable='autodrive_bridge',
            name='autodrive_bridge',
            emulate_tty=True,
            output='screen',
        ),
        localize,
        Node(
            package='autodrive_roboracer',
            executable='amcl_initialpose',
            name='amcl_initialpose',
            emulate_tty=True,
            output='screen',
            parameters=[{'map_name': map_name}],
        ),
        OpaqueFunction(function=_controller_node),
    ])
