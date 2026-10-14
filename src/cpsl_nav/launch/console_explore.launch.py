"""
Exploration for one robot, as console runs it: frontier exploration or wander.

mode:=frontier  cpsl_nav frontier_explore, node /<namespace>/frontier_explore
mode:=wander    cpsl_nav wander, node /<namespace>/wander

Both need Nav2 (console_nav.launch.py) and a map (console_slam.launch.py,
either mode) in the same namespace, send their own NavigateToPose goals, and
publish JSON status on ``/<namespace>/<node>/status``. SIGINT cancels the
active goal before the node exits.

tf is read on `tf_topic` (default /<namespace>/tf, as console_slam.launch.py
publishes map -> odom) and `tf_static_topic` (default /tf_static).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = {'frontier': 'frontier_explore', 'wander': 'wander'}

ARGUMENTS = [
    DeclareLaunchArgument('mode', default_value='frontier', choices=list(NODES),
                          description='frontier: frontier exploration; wander: random '
                                      'reachable goals'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace (the one Nav2 runs in)'),
    DeclareLaunchArgument('map_topic', default_value='map',
                          description='OccupancyGrid to explore (relative = in the namespace)'),
    DeclareLaunchArgument('keepout_topic', default_value='keepout_filter_mask',
                          description="Keepout mask (console_nav.launch.py's filter_mask_server)"),
    DeclareLaunchArgument('tf_topic', default_value='',
                          description='tf topic (default /<namespace>/tf, or /tf with no '
                                      'namespace)'),
    DeclareLaunchArgument('tf_static_topic', default_value='/tf_static',
                          description='Static tf topic'),
    DeclareLaunchArgument('base_frame', default_value='base_link', description='Robot base frame'),
    DeclareLaunchArgument('map_frame', default_value='map', description='Map frame'),
    DeclareLaunchArgument('goal_timeout', default_value='90.0',
                          description='Seconds before a goal is cancelled as failed'),
    DeclareLaunchArgument('seed', default_value='-1',
                          description='wander: random seed (-1 = from the clock)'),
    DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                          description='Use /clock'),
    DeclareLaunchArgument('log_level', default_value='info', description='Node log level'),
]


def _arg(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def launch_setup(context, *args, **kwargs):
    mode = _arg(context, 'mode')
    namespace = _arg(context, 'namespace').strip('/')
    tf_topic = _arg(context, 'tf_topic') or (f'/{namespace}/tf' if namespace else '/tf')
    tf_static_topic = _arg(context, 'tf_static_topic') or '/tf_static'
    if not tf_topic.startswith('/') or not tf_static_topic.startswith('/'):
        raise RuntimeError('tf_topic and tf_static_topic must be absolute topic names')
    params = {
        'map_topic': _arg(context, 'map_topic') or 'map',
        'keepout_topic': _arg(context, 'keepout_topic') or 'keepout_filter_mask',
        'base_frame': _arg(context, 'base_frame'),
        'map_frame': _arg(context, 'map_frame'),
        'goal_timeout': float(_arg(context, 'goal_timeout')),
        'use_sim_time': _arg(context, 'use_sim_time') == 'true',
    }
    if mode == 'wander':
        params['seed'] = int(_arg(context, 'seed'))
    return [Node(
        package='cpsl_nav', executable=NODES[mode], name=NODES[mode],
        namespace='/' + namespace if namespace else '', output='screen',
        arguments=['--ros-args', '--log-level', _arg(context, 'log_level')],
        parameters=[params],
        remappings=[('/tf', tf_topic), ('/tf_static', tf_static_topic)],
        # The node cancels its goal on SIGINT; give it time before SIGTERM.
        sigterm_timeout='5')]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
