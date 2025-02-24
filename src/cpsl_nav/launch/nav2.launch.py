from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap, Node

pkg_cpsl_navigation = get_package_share_directory('cpsl_nav')
pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('params_file',
                          default_value=PathJoinSubstitution([
                              pkg_cpsl_navigation,
                              'config',
                              'nav2.yaml'
                              ]),
                          description='Nav2 parameters'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('scan_topic', default_value='/scan',
                          description='The LaserScan topic to use for slam'),
]

def launch_setup(context, *args, **kwargs):

    nav2_params = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')

    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str

    scan_topic_str = scan_topic.perform(context)

    launch_nav2 = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = GroupAction([
        PushRosNamespace(namespace),

        SetRemap(namespace_str + '/global_costmap/scan', namespace_str + scan_topic_str),
        SetRemap(namespace_str + '/local_costmap/scan', namespace_str + scan_topic_str),
        SetRemap('/local_costmap/published_footprint', namespace_str + '/local_costmap/published_footprint'),
        SetRemap('/trajectories', namespace_str + '/trajectories'),
        SetRemap('/tf', namespace_str + '/tf'),
        SetRemap('/tf_static', namespace_str + '/tf_static'),
        SetRemap(namespace_str + '/scan', namespace_str + scan_topic_str),
        SetRemap('/map', namespace_str + '/map'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                  ('use_sim_time', use_sim_time),
                  ('params_file', nav2_params.perform(context)),
                  ('use_composition', 'False'),
                  ('namespace', namespace_str)
                ]
        ),
    ])

    return [nav2]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld