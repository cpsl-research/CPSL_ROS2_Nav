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
pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('config', default_value='localization_config.rviz',
                          description='rviz config file in the rviz_cfgs directory')
]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = LaunchConfiguration('config')

    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str
    
    config_file_str = config.perform(context)
    
    rviz_config_file = PathJoinSubstitution([pkg_cpsl_navigation, 'rviz_cfgs', config_file_str])

    # Apply the following re-mappings only within this group
    rviz = GroupAction([
        PushRosNamespace(namespace),

        SetRemap('/tf', namespace_str + '/tf'),
        SetRemap('/tf_static', namespace_str + '/tf_static'),
        SetRemap('/initialpose', namespace_str + '/initialpose'),
        SetRemap('/goal_pose', namespace_str + '/goal_pose'),
        # SetRemap('/scan', namespace_str + scan_topic_str),
        # SetRemap('/map', namespace_str + '/map'),
        # SetRemap('/map_metadata', namespace_str + '/map_metadata'),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])

    return [rviz]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
