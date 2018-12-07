import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions
from typing import Dict
from typing import List
from typing import Text
import yaml
import tempfile

class RewrittenYaml(launch.Substitution):
    """
    Substitution that modifies the given Yaml file.
    """

    def __init__(self, source_file: launch.SomeSubstitutionsType, rewrites: Dict) -> None:
        super().__init__()

        from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__rewrites = rewrites

    @property
    def name(self) -> List[launch.Substitution]:
        """Getter for name."""
        return self.__source_file

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: launch.LaunchContext) -> Text:
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
        resolved_rewrites = self.resolve_rewrites(context)
        data = yaml.load(open(yaml_filename, 'r'))
        self.substitute_values(data, resolved_rewrites)
        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def resolve_rewrites(self, context):
        return self.__rewrites

    def substitute_values(self, yaml, rewrites):
        return

def generate_launch_description():
    map_yaml_file = launch.substitutions.LaunchConfiguration('map')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    params_file = launch.substitutions.LaunchConfiguration('params', default=
        [launch.substitutions.ThisLaunchFileDir(), '/nav2_params.yaml'])
    final_params = RewrittenYaml(params_file, {'use_sim_time': False})

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'map', description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']),

        launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan']),

        launch_ros.actions.Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[final_params, { 'yaml_filename': map_yaml_file }]),

        launch_ros.actions.Node(
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[final_params]),

        launch_ros.actions.Node(
            package='nav2_amcl',
            node_executable='amcl',
            node_name='amcl',
            output='screen',
            parameters=[final_params]),

        launch_ros.actions.Node(
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[final_params]),

        launch_ros.actions.Node(
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            node_name='navfn_planner',
            output='screen',
            parameters=[final_params]),

        launch_ros.actions.Node(
            package='nav2_simple_navigator',
            node_executable='simple_navigator',
            node_name='simple_navigator',
            output='screen',
            parameters=[final_params]),

        launch_ros.actions.Node(
            package='nav2_mission_executor',
            node_executable='mission_executor',
            node_name='mission_executor',
            output='screen',
            parameters=[final_params]),
    ])
