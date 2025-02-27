# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable,OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


pkg_cpsl_navigation = get_package_share_directory('cpsl_nav')
pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace', 
        default_value='', 
        description='Top-level namespace'
    ),
    DeclareLaunchArgument('scan_topic', 
        default_value='/scan',
        description='The LaserScan topic to use for slam'),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    ),
    DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_cpsl_navigation, 'config', 'nav2.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    ),
    DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    ),
    DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    ),
    DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of container that nodes will load in if use composition',
    ),
    DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    ),
    DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )
]

def launch_setup(context, *args,**kwargs):
    # Get the launch directory

    namespace = LaunchConfiguration('namespace')
    scan_topic = LaunchConfiguration('scan_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
        # 'docking_server',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str


    scan_topic_str = scan_topic.perform(context)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            SetRemap(namespace_str + '/scan', namespace_str + scan_topic_str),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
                + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # Node(
            #     package='opennav_docking',
            #     executable='opennav_docking',
            #     name='docking_server',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    # load_composable_nodes = GroupAction(
    #     condition=IfCondition(use_composition),
    #     actions=[
    #         SetParameter('use_sim_time', use_sim_time),
    #         LoadComposableNodes(
    #             target_container=container_name_full,
    #             composable_node_descriptions=[
    #                 ComposableNode(
    #                     package='nav2_controller',
    #                     plugin='nav2_controller::ControllerServer',
    #                     name='controller_server',
    #                     parameters=[configured_params],
    #                     remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    #                 ),
    #                 ComposableNode(
    #                     package='nav2_smoother',
    #                     plugin='nav2_smoother::SmootherServer',
    #                     name='smoother_server',
    #                     parameters=[configured_params],
    #                     remappings=remappings,
    #                 ),
    #                 ComposableNode(
    #                     package='nav2_planner',
    #                     plugin='nav2_planner::PlannerServer',
    #                     name='planner_server',
    #                     parameters=[configured_params],
    #                     remappings=remappings,
    #                 ),
    #                 ComposableNode(
    #                     package='nav2_behaviors',
    #                     plugin='behavior_server::BehaviorServer',
    #                     name='behavior_server',
    #                     parameters=[configured_params],
    #                     remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    #                 ),
    #                 ComposableNode(
    #                     package='nav2_bt_navigator',
    #                     plugin='nav2_bt_navigator::BtNavigator',
    #                     name='bt_navigator',
    #                     parameters=[configured_params],
    #                     remappings=remappings,
    #                 ),
    #                 ComposableNode(
    #                     package='nav2_waypoint_follower',
    #                     plugin='nav2_waypoint_follower::WaypointFollower',
    #                     name='waypoint_follower',
    #                     parameters=[configured_params],
    #                     remappings=remappings,
    #                 ),
    #                 ComposableNode(
    #                     package='nav2_velocity_smoother',
    #                     plugin='nav2_velocity_smoother::VelocitySmoother',
    #                     name='velocity_smoother',
    #                     parameters=[configured_params],
    #                     remappings=remappings
    #                     + [('cmd_vel', 'cmd_vel_nav')],
    #                 ),
    #                 ComposableNode(
    #                     package='nav2_collision_monitor',
    #                     plugin='nav2_collision_monitor::CollisionMonitor',
    #                     name='collision_monitor',
    #                     parameters=[configured_params],
    #                     remappings=remappings,
    #                 ),
    #                 ComposableNode(
    #                     package='opennav_docking',
    #                     plugin='opennav_docking::DockingServer',
    #                     name='docking_server',
    #                     parameters=[configured_params],
    #                     remappings=remappings,
    #                 ),
    #                 ComposableNode(
    #                     package='nav2_lifecycle_manager',
    #                     plugin='nav2_lifecycle_manager::LifecycleManager',
    #                     name='lifecycle_manager_navigation',
    #                     parameters=[
    #                         {'autostart': autostart, 'node_names': lifecycle_nodes}
    #                     ],
    #                 ),
    #             ],
    #         ),
    #     ],
    # )

    

    return [load_nodes]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
