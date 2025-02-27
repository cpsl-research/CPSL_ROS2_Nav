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
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Use synchronous SLAM'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('scan_topic', default_value='/scan',
                          description='The LaserScan topic to use for slam'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.'),
    DeclareLaunchArgument('use_lifecycle_manager', default_value='false',
                          choices=['true', 'false'],
                          description='Enable bond connection during node activation'),
    DeclareLaunchArgument('slam_params_file',
                          default_value='slam.yaml',
                          description='SLAM YAML file in the config folder'),
    DeclareLaunchArgument('rviz',
                          default_value='false',
                          choices=['true','false'],
                          description='Display an RViz window with navigation')
]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    scan_topic = LaunchConfiguration('scan_topic')
    sync = LaunchConfiguration('sync')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    slam_params = LaunchConfiguration('slam_params_file')
    rviz = LaunchConfiguration('rviz')

    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str

    launch_slam_sync = PathJoinSubstitution(
        [pkg_slam_toolbox, 'launch', 'online_sync_launch.py'])

    launch_slam_async = PathJoinSubstitution(
        [pkg_slam_toolbox, 'launch', 'online_async_launch.py'])
    
    scan_topic_str = scan_topic.perform(context)
    
    rviz_config_file = PathJoinSubstitution([pkg_cpsl_navigation, 'rviz_cfgs', 'slam_config.rviz'])

    slam_params_str = slam_params.perform(context)
    slam_config_file = PathJoinSubstitution([pkg_cpsl_navigation, 'config', slam_params_str])

    # Apply the following re-mappings only within this group
    slam = GroupAction([
        PushRosNamespace(namespace),

        SetRemap('/tf', namespace_str + '/tf'),
        SetRemap('/tf_static', namespace_str + '/tf_static'),
        SetRemap('/scan', namespace_str + scan_topic_str),
        SetRemap('/map', namespace_str + '/map'),
        SetRemap('/map_metadata', namespace_str + '/map_metadata'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam_sync),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('autostart', autostart),
                ('use_lifecycle_manager', use_lifecycle_manager),
                ('slam_params_file', slam_config_file)
            ],
            condition=IfCondition(sync)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam_async),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('autostart', autostart),
                ('use_lifecycle_manager', use_lifecycle_manager),
                ('slam_params_file', slam_config_file)
            ],
            condition=UnlessCondition(sync)
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz)
        )
    ])

    return [slam]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
