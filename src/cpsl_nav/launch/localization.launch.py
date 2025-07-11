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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (EqualsSubstitution, LaunchConfiguration, NotEqualsSubstitution,
                                  PythonExpression, PathJoinSubstitution)
from launch_ros.actions import LoadComposableNodes, Node, SetParameter, PushRosNamespace
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


pkg_cpsl_navigation = get_package_share_directory('cpsl_nav')
pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    ),
    DeclareLaunchArgument(
        'map',
        default_value='cpsl.yaml',
        description='yaml file in the cpsl_nav/maps folder with map information'
    ),
    DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='The LaserScan topic to use for slam (without the namespace)'),
    DeclareLaunchArgument(
        'base_frame_id', default_value='base_link',
        description='The frame ID of the base_link frame (without tf pre-fix)'),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    ),
    DeclareLaunchArgument(
        'params_file',
        default_value='localization.yaml',
        description='Full path to the ROS2 parameters file in the cpsl_nav/config folder',
    ),
    DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
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

def launch_setup(context, *args, **kwargs):

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame_id = LaunchConfiguration('base_frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')


    namespace_str = namespace.perform(context)
    base_frame_id_str = base_frame_id.perform(context)
    if (namespace_str):
        if not namespace_str.startswith('/'):
            namespace_str = '/' + namespace_str
        tf_prefix = namespace_str.strip("/")
        odom_frame = "{}/odom".format(tf_prefix)
        base_frame = "{}/{}".format(tf_prefix,base_frame_id_str)
    else:
        tf_prefix = ""
        odom_frame = "odom"
        base_frame = base_frame_id_str

    map_path = PathJoinSubstitution(
        [pkg_cpsl_navigation,'maps',map_yaml_file]
    )
   
    scan_topic_str = scan_topic.perform(context)
    scan_topic_str = scan_topic_str.strip("/")

    lifecycle_nodes = ['map_server', 'amcl']

    # Create our own temporary YAML files that include substitutions
    params_str = params_file.perform(context)
    nav_config_file = PathJoinSubstitution([pkg_cpsl_navigation, 'config', params_str])

    #substituting parameters
    param_substitutions = {
        'scan_topic':scan_topic_str,
        'odom_frame_id':odom_frame,
        'base_frame_id':base_frame
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav_config_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    remappings = [
        # ('/tf', 'tf'),
        # ('/tf_static', 'tf_static'),
        ('initialpose', '/initialpose'),
    ]

    load_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushRosNamespace(namespace),
            Node(
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                condition=IfCondition(
                    NotEqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_path}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    return [load_nodes]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld