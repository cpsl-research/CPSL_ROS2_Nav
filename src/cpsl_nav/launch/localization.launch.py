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
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('scan_topic', default_value='/scan',
                          description='The LaserScan topic to use for slam'),
    DeclareLaunchArgument('param_file',
                          default_value='localization.yaml',
                          description='localization YAML file in the config folder'),
    DeclareLaunchArgument('map', 
                            default_value='cpsl.yaml',
                            description='yaml file in the cpsl_nav/maps folder with map information'),
]

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    scan_topic = LaunchConfiguration('scan_topic')
    params_file = LaunchConfiguration('param_file')
    map = LaunchConfiguration('map')

    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str

    map_path = PathJoinSubstitution(
        [pkg_cpsl_navigation,'maps',map]
    )
   
    scan_topic_str = scan_topic.perform(context)

    params_file_str = params_file.perform(context)
    param_file = PathJoinSubstitution([pkg_cpsl_navigation, 'config', params_file_str])
    
    # Apply the following re-mappings only within this group
    localization = GroupAction([
        PushRosNamespace(namespace),

        # SetRemap('/tf', namespace_str + '/tf'),
        # SetRemap('/tf_static', namespace_str + '/tf_static'),
        SetRemap(namespace_str + '/scan', namespace_str + scan_topic_str),
        SetRemap('/scan',scan_topic_str),
        # SetRemap('/initialpose', namespace_str + '/initialpose'),
        # SetRemap('/goal_pose', namespace_str + '/goal_posetmux'),
        # SetRemap('/map', namespace_str + '/map'),
        # SetRemap('/map_metadata', namespace_str + '/map_metadata'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_nav2_bringup, 'launch', 'localization_launch.py'])),
            launch_arguments=[('namespace',namespace),
                              ('map', map_path),
                              ('use_sim_time', use_sim_time),
                              ('params_file', param_file)],
        )
    ])

    return [localization]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
