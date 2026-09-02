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
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_bringup.keepout_zone_launch import get_lifecycle_nodes as get_keepout_zone_nodes
from nav2_bringup.localization_launch import get_lifecycle_nodes as get_localization_nodes
from nav2_bringup.navigation_launch import get_lifecycle_nodes as get_navigation_nodes
from nav2_bringup.slam_launch import get_lifecycle_nodes as get_slam_nodes
from nav2_bringup.speed_zone_launch import get_lifecycle_nodes as get_speed_zone_nodes
from nav2_common.launch import LaunchConfigAsBool, RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    slam = LaunchConfigAsBool('slam')
    map_yaml_file = LaunchConfiguration('map')
    keepout_mask_yaml_file = LaunchConfiguration('keepout_mask')
    speed_mask_yaml_file = LaunchConfiguration('speed_mask')
    graph_filepath = LaunchConfiguration('graph')
    use_sim_time = LaunchConfigAsBool('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfigAsBool('autostart')
    use_composition = LaunchConfigAsBool('use_composition')
    use_intra_process_comms = LaunchConfigAsBool('use_intra_process_comms')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfigAsBool('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_amcl = LaunchConfigAsBool('use_amcl')
    use_keepout_zones = LaunchConfigAsBool('use_keepout_zones')
    use_speed_zones = LaunchConfigAsBool('use_speed_zones')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    yaml_substitutions = {
        'KEEPOUT_ZONE_ENABLED': use_keepout_zones,
        'SPEED_ZONE_ENABLED': use_speed_zones,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            value_rewrites=yaml_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    def launch_lifecycle_manager(context):
        slam_enabled = slam.perform(context) == 'True'
        amcl_enabled = use_amcl.perform(context) == 'True'
        keepout_zones_enabled = use_keepout_zones.perform(context) == 'True'
        speed_zones_enabled = use_speed_zones.perform(context) == 'True'

        lifecycle_nodes = []

        if slam_enabled:
            lifecycle_nodes.extend(get_slam_nodes())
        else:
            lifecycle_nodes.extend(get_localization_nodes(use_amcl=amcl_enabled))

        if keepout_zones_enabled:
            lifecycle_nodes.extend(get_keepout_zone_nodes())

        if speed_zones_enabled:
            lifecycle_nodes.extend(get_speed_zone_nodes())

        lifecycle_nodes.extend(get_navigation_nodes())

        manager_parameters = [
            configured_params,
            {
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'use_sim_time': use_sim_time,
            },
        ]

        return [
            LoadComposableNodes(
                condition=IfCondition(use_composition),
                target_container=(namespace, '/', container_name),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_nav2',
                        namespace=namespace,
                        parameters=manager_parameters,
                        extra_arguments=[{
                            'use_intra_process_comms': use_intra_process_comms
                        }],
                    ),
                ],
            ),
            Node(
                condition=UnlessCondition(use_composition),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_nav2',
                namespace=namespace,
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=manager_parameters,
            ),
        ]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value='', description='Full path to map yaml file to load'
    )

    declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
        'keepout_mask', default_value='',
        description='Full path to keepout mask yaml file to load'
    )

    declare_speed_mask_yaml_cmd = DeclareLaunchArgument(
        'speed_mask', default_value='',
        description='Full path to speed mask yaml file to load'
    )

    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value='', description='Path to the graph file to load'
    )

    declare_use_amcl_cmd = DeclareLaunchArgument(
        'use_amcl', default_value='True',
        description='Whether to enable AMCL when using localization'
    )

    declare_use_keepout_zones_cmd = DeclareLaunchArgument(
        'use_keepout_zones', default_value='True',
        description='Whether to enable keepout zones or not'
    )

    declare_use_speed_zones_cmd = DeclareLaunchArgument(
        'use_speed_zones', default_value='True',
        description='Whether to enable speed zones or not'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_intra_process_comms_cmd = DeclareLaunchArgument(
        'use_intra_process_comms',
        default_value='False',
        description='Whether to use intra process communications',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of container that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            Node(
                condition=IfCondition(use_composition),
                name=container_name,
                namespace=namespace,
                package='rclcpp_components',
                executable='component_container',
                parameters=[configured_params, {'autostart': autostart}],
                arguments=['--isolated', '--executor-type', 'single-threaded',
                           '--ros-args', '--log-level', log_level],
                remappings=remappings,
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'slam_launch.py')
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'use_respawn': use_respawn,
                    'params_file': params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'localization_launch.py')
                ),
                condition=IfCondition(PythonExpression(['not ', slam])),
                launch_arguments={
                    'namespace': namespace,
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'use_amcl': use_amcl,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_intra_process_comms': use_intra_process_comms,
                    'use_respawn': use_respawn,
                    'container_name': container_name,
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'keepout_zone_launch.py')
                ),
                condition=IfCondition(use_keepout_zones),
                launch_arguments={
                    'namespace': namespace,
                    'keepout_mask': keepout_mask_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_intra_process_comms': use_intra_process_comms,
                    'use_respawn': use_respawn,
                    'container_name': container_name,
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'speed_zone_launch.py')
                ),
                condition=IfCondition(use_speed_zones),
                launch_arguments={
                    'namespace': namespace,
                    'speed_mask': speed_mask_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_intra_process_comms': use_intra_process_comms,
                    'use_respawn': use_respawn,
                    'container_name': container_name,
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'graph': graph_filepath,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_intra_process_comms': use_intra_process_comms,
                    'use_respawn': use_respawn,
                    'use_keepout_zones': use_keepout_zones,
                    'use_speed_zones': use_speed_zones,
                    'container_name': container_name,
                }.items(),
            ),
            OpaqueFunction(function=launch_lifecycle_manager),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_keepout_mask_yaml_cmd)
    ld.add_action(declare_speed_mask_yaml_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_intra_process_comms_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_amcl_cmd)
    ld.add_action(declare_use_keepout_zones_cmd)
    ld.add_action(declare_use_speed_zones_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
