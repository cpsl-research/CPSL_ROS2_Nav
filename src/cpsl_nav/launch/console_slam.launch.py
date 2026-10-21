"""
Mapping or localization for one robot, as console runs it.

mode:=mapping       slam_toolbox (online async), lifecycle configure + activate
                    done here, publishing the map on <namespace>/map and
                    map -> <odom_frame> on `tf_topic`.
mode:=localization  map_server serving `map_yaml` on <namespace>/map, and AMCL
                    on it, both brought up by a nav2 lifecycle manager. AMCL
                    waits for an initial pose on <namespace>/initialpose, then
                    publishes map -> <odom_frame> on `tf_topic`.

Frames and tf, the part that decides whether anything works. Every node here
reads and writes tf on `tf_topic` (default relative `tf`, i.e. /<namespace>/tf,
or /tf with no namespace) and static tf on `tf_static_topic` (default relative
`tf_static`): see console's docs/TF_AND_TOPICS.md. The
remaps are explicit because stock launch files silently stay on /tf. Frame
names are passed as given (no tf prefix is added): the Create 3 publishes
`odom -> base_link`, unprefixed, on /<namespace>/tf.

Parameters: `params_file` (default: this package's config/slam_<platform>.yaml)
merged with `overrides_file`, then rewritten by cpsl_nav/params.py into a
temporary file; see docs/DEVELOPING.md, "How a console launch runs" and "`params.py` rules".
"""

import os

from ament_index_python.packages import get_package_share_directory
from cpsl_nav import params as cparams
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

ARGUMENTS = [
    DeclareLaunchArgument('mode', default_value='mapping',
                          choices=['mapping', 'localization'],
                          description='mapping: slam_toolbox; localization: '
                                      'map_server + AMCL on map_yaml'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace; every node runs in it'),
    DeclareLaunchArgument('platform', default_value='create3',
                          description='console platform kind; selects '
                                      'config/slam_<platform>.yaml when params_file is empty'),
    DeclareLaunchArgument('params_file', default_value='',
                          description='Full path of the platform parameter file '
                                      '(default: installed config/slam_<platform>.yaml)'),
    DeclareLaunchArgument('overrides_file', default_value='',
                          description='Optional YAML merged over params_file '
                                      '(same shape; existing keys only). '
                                      'Empty or missing: no overrides'),
    DeclareLaunchArgument('tf_topic', default_value='tf',
                          description='tf topic the nodes read and publish on; relative '
                                      'names resolve in the namespace (/<namespace>/tf)'),
    DeclareLaunchArgument('tf_static_topic', default_value='tf_static',
                          description='Static tf topic the nodes read; relative names '
                                      'resolve in the namespace'),
    DeclareLaunchArgument('scan_topic', default_value='scan',
                          description='LaserScan topic (relative names resolve in the namespace)'),
    DeclareLaunchArgument('cloud_topic', default_value='',
                          description='Optional PointCloud2: if set, pointcloud_to_laserscan '
                                      'flattens it onto scan_topic'),
    DeclareLaunchArgument('base_frame', default_value='base_link',
                          description='Robot base frame, as published (no prefix added)'),
    DeclareLaunchArgument('odom_frame', default_value='odom',
                          description='Odometry frame, as published (no prefix added)'),
    DeclareLaunchArgument('map_frame', default_value='map', description='Map frame'),
    DeclareLaunchArgument('map_yaml', default_value='',
                          description='localization: full path of the map_server YAML to serve'),
    DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                          description='Use /clock'),
    DeclareLaunchArgument('log_level', default_value='info', description='Node log level'),
]


def _arg(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def launch_setup(context, *args, **kwargs):
    mode = _arg(context, 'mode')
    namespace = _arg(context, 'namespace').strip('/')
    platform = _arg(context, 'platform')
    params_file = _arg(context, 'params_file')
    overrides_file = _arg(context, 'overrides_file')
    tf_topic = _arg(context, 'tf_topic') or 'tf'
    tf_static_topic = _arg(context, 'tf_static_topic') or 'tf_static'
    scan_topic = _arg(context, 'scan_topic') or 'scan'
    cloud_topic = _arg(context, 'cloud_topic')
    base_frame = _arg(context, 'base_frame')
    odom_frame = _arg(context, 'odom_frame')
    map_frame = _arg(context, 'map_frame')
    map_yaml = _arg(context, 'map_yaml')
    use_sim_time = _arg(context, 'use_sim_time') == 'true'
    log_level = _arg(context, 'log_level')

    if not params_file:
        params_file = cparams.platform_file(
            os.path.join(get_package_share_directory('cpsl_nav'), 'config'), platform)
    elif not os.path.isfile(params_file):
        raise RuntimeError(f'params_file not found: {params_file}')

    sets = cparams.slam_sets(mode, namespace, scan_topic, base_frame, odom_frame,
                             map_frame, map_yaml, use_sim_time)
    effective = cparams.write(
        cparams.effective(params_file, overrides_file, namespace, sets),
        prefix=f'console_slam_{namespace or "root"}_')

    ns = '/' + namespace if namespace else ''
    remaps = [('/tf', tf_topic), ('/tf_static', tf_static_topic)]
    ros_args = ['--ros-args', '--log-level', log_level]
    actions = [
        LogInfo(msg=f'[console_slam] mode={mode} namespace={ns or "/"} tf={tf_topic} '
                    f'tf_static={tf_static_topic} scan={cparams.topic(namespace, scan_topic)} '
                    f'frames={map_frame}->{odom_frame}->{base_frame}'),
        LogInfo(msg=f'[console_slam] params={params_file} overrides='
                    f'{overrides_file if os.path.isfile(overrides_file) else "(none)"} '
                    f'effective={effective}'),
    ]

    if cloud_topic:
        # Same settings as console's mapper used (from the generated sensors
        # launch), target frame = base frame.
        actions.append(Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='scan_converter', namespace=ns, output='screen', arguments=ros_args,
            remappings=remaps + [('cloud_in', cloud_topic), ('scan', scan_topic)],
            parameters=[{
                'min_height': -0.1, 'max_height': 0.1,
                'angle_min': -3.141592653589793, 'angle_max': 3.141592653589793,
                'angle_increment': 0.0174532925, 'queue_size': 10, 'scan_time': 0.05,
                'range_min': 0.25, 'range_max': 5.0, 'target_frame': base_frame,
                'transform_tolerance': 0.01, 'use_inf': True, 'use_sim_time': use_sim_time,
            }]))

    if mode == 'mapping':
        slam = LifecycleNode(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', namespace=ns, output='screen', arguments=ros_args,
            parameters=[effective, {'use_lifecycle_manager': False}],
            remappings=remaps)
        actions += [
            slam,
            # The node waits in `unconfigured`; drive it to `active` here, as
            # slam_toolbox's own online_async_launch.py does.
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_CONFIGURE)),
            RegisterEventHandler(OnStateTransition(
                target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
                entities=[
                    LogInfo(msg='[console_slam] slam_toolbox configured; activating'),
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE)),
                ])),
            RegisterEventHandler(OnStateTransition(
                target_lifecycle_node=slam, start_state='activating', goal_state='active',
                entities=[LogInfo(msg='[console_slam] mapping: slam_toolbox active')])),
        ]
    else:
        # nav2 lifecycle manager: configures then activates map_server, then
        # AMCL (AMCL needs the map), and holds bonds to both.
        actions += [
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 namespace=ns, output='screen', arguments=ros_args,
                 parameters=[effective], remappings=remaps),
            Node(package='nav2_amcl', executable='amcl', name='amcl',
                 namespace=ns, output='screen', arguments=ros_args,
                 parameters=[effective], remappings=remaps),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', namespace=ns, output='screen',
                 arguments=ros_args,
                 parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl'],
                              'use_sim_time': use_sim_time}]),
        ]
    # The nodes read the effective file at startup; remove it when the launch ends.
    actions.append(RegisterEventHandler(OnShutdown(
        on_shutdown=[OpaqueFunction(function=lambda _ctx: _remove(effective))])))
    return actions


def _remove(path):
    try:
        os.unlink(path)
    except OSError:
        pass


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
