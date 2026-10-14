"""
Nav2 for one robot, as console runs it, with a keepout filter (no-go zones).

Runs, in the robot's namespace:

* stock nav2_bringup ``bringup_launch.py``, navigation only (no slam, no
  localization: map -> odom comes from console_slam.launch.py). bringup, not
  navigation_launch.py, because only bringup pushes the namespace onto the
  nodes;
* the keepout filter's ``costmap_filter_info_server`` and
  ``filter_mask_server`` (a map_server serving the mask), brought up by
  ``lifecycle_manager_costmap_filters``. Both costmaps of every
  ``nav2_<platform>.yaml`` carry ``filters: [keepout_filter]``. No
  ``keepout_mask`` = a one-cell free mask, i.e. no zones. A new mask is
  loaded at run time with ``/<ns>/filter_mask_server/load_map``;
* ``tf_static_relay`` (with a namespace): copies /tf_static onto
  /<ns>/tf_static.

tf: stock bringup remaps /tf and /tf_static to the relative ``tf`` and
``tf_static``, so every Nav2 node reads /<ns>/tf (where the robot's
odom -> base_link and the mapper's map -> odom are, matching
console_slam.launch.py's default tf_topic) and /<ns>/tf_static (fed by the
relay).

Parameters: `params_file` (default: this package's config/nav2_<platform>.yaml)
merged with `overrides_file`, then rewritten by cpsl_nav/params.py
(nav2_rewrite: frames, scan/map/odom/cmd_vel topics, keepout topics) into a
temporary file; see the README, "Parameters for console".
"""

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from cpsl_nav import params as cparams
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace; every node runs in it'),
    DeclareLaunchArgument('platform', default_value='create3',
                          description='console platform kind; selects '
                                      'config/nav2_<platform>.yaml when params_file is empty'),
    DeclareLaunchArgument('params_file', default_value='',
                          description='Full path of the platform Nav2 parameter file '
                                      '(default: installed config/nav2_<platform>.yaml)'),
    DeclareLaunchArgument('overrides_file', default_value='',
                          description='Optional YAML merged over params_file '
                                      '(same shape; existing keys only). '
                                      'Empty or missing: no overrides'),
    DeclareLaunchArgument('scan_topic', default_value='scan',
                          description='LaserScan for the costmaps and the collision monitor '
                                      '(relative names resolve in the namespace)'),
    DeclareLaunchArgument('map_topic', default_value='map',
                          description='OccupancyGrid for the static layers '
                                      "(default /<namespace>/map, console_slam's)"),
    DeclareLaunchArgument('odom_topic', default_value='',
                          description="Odometry topic; empty keeps the platform file's"),
    DeclareLaunchArgument('cmd_vel_topic', default_value='',
                          description='Command output (collision monitor); empty keeps '
                                      "the platform file's"),
    DeclareLaunchArgument('base_frame', default_value='base_link',
                          description='Robot base frame, as published (no prefix added)'),
    DeclareLaunchArgument('odom_frame', default_value='odom',
                          description='Odometry frame, as published (no prefix added)'),
    DeclareLaunchArgument('map_frame', default_value='map', description='Map frame'),
    DeclareLaunchArgument('keepout_mask', default_value='',
                          description='Full path of the keepout mask YAML (map_server format, '
                                      'black = keepout). Empty: no zones'),
    DeclareLaunchArgument('tf_static_relay', default_value='true', choices=['true', 'false'],
                          description='With a namespace, copy /tf_static onto '
                                      '/<namespace>/tf_static'),
    DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                          description='Use /clock'),
    DeclareLaunchArgument('log_level', default_value='info', description='Node log level'),
]


def _arg(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def launch_setup(context, *args, **kwargs):
    namespace = _arg(context, 'namespace').strip('/')
    platform = _arg(context, 'platform')
    params_file = _arg(context, 'params_file')
    overrides_file = _arg(context, 'overrides_file')
    scan_topic = _arg(context, 'scan_topic') or 'scan'
    map_topic = _arg(context, 'map_topic') or 'map'
    keepout_mask = _arg(context, 'keepout_mask')
    use_sim_time = _arg(context, 'use_sim_time') == 'true'
    log_level = _arg(context, 'log_level')
    frames = {k: _arg(context, k) for k in ('base_frame', 'odom_frame', 'map_frame')}

    share = get_package_share_directory('cpsl_nav')
    if not params_file:
        params_file = cparams.platform_file(os.path.join(share, 'config'), platform,
                                            stem='nav2')
    elif not os.path.isfile(params_file):
        raise RuntimeError(f'params_file not found: {params_file}')

    tmp = tempfile.mkdtemp(prefix=f'console_nav_{namespace or "root"}_')
    nav2 = cparams.write(cparams.nav2_effective(
        params_file, overrides_file, namespace=namespace, scan_topic=scan_topic,
        map_topic=map_topic, odom_topic=_arg(context, 'odom_topic'),
        cmd_vel_topic=_arg(context, 'cmd_vel_topic'), use_sim_time=use_sim_time, **frames),
        directory=tmp, prefix='nav2_')
    mask = keepout_mask or cparams.write_empty_mask(tmp)
    keepout = cparams.write(cparams.rewrite(
        cparams.load(os.path.join(share, 'config', 'keepout_filter.yaml')), namespace,
        cparams.keepout_sets(namespace, mask, frames['map_frame'], use_sim_time)),
        directory=tmp, prefix='keepout_')

    ns = '/' + namespace if namespace else ''
    remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    ros_args = ['--ros-args', '--log-level', log_level]
    bool_str = {True: 'True', False: 'False'}
    actions = [
        LogInfo(msg=f'[console_nav] namespace={ns or "/"} tf={ns}/tf tf_static={ns}/tf_static '
                    f'scan={cparams.topic(namespace, scan_topic)} '
                    f'map={cparams.topic(namespace, map_topic)} '
                    f'frames={frames["map_frame"]}->{frames["odom_frame"]}'
                    f'->{frames["base_frame"]}'),
        LogInfo(msg=f'[console_nav] params={params_file} overrides='
                    f'{overrides_file if os.path.isfile(overrides_file) else "(none)"} '
                    f'effective={nav2} keepout_mask={keepout_mask or "(none)"}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace, 'use_namespace': bool_str[bool(namespace)],
                'slam': 'False', 'use_localization': 'False', 'use_composition': 'False',
                'params_file': nav2, 'use_sim_time': bool_str[use_sim_time],
                'autostart': 'True', 'log_level': log_level,
            }.items()),
        Node(package='nav2_map_server', executable='map_server', name='filter_mask_server',
             namespace=ns, output='screen', arguments=ros_args, parameters=[keepout],
             remappings=remaps),
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name='costmap_filter_info_server', namespace=ns, output='screen',
             arguments=ros_args, parameters=[keepout], remappings=remaps),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_costmap_filters', namespace=ns, output='screen',
             arguments=ros_args,
             parameters=[{'autostart': True, 'use_sim_time': use_sim_time,
                          'node_names': ['filter_mask_server', 'costmap_filter_info_server']}]),
    ]
    if namespace and _arg(context, 'tf_static_relay') == 'true':
        actions.append(Node(package='cpsl_nav', executable='tf_static_relay',
                            name='tf_static_relay', namespace=ns, output='screen',
                            arguments=[f'{ns}/tf_static'] + ros_args))
    # The nodes read the files at startup; remove them when the launch ends.
    actions.append(RegisterEventHandler(OnShutdown(
        on_shutdown=[OpaqueFunction(function=lambda _ctx: shutil.rmtree(tmp, True))])))
    return actions


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
