import os
import yaml
import tempfile
from launch import LaunchDescription
import launch.actions
import launch_ros.actions
from typing import Dict
from typing import List
from typing import Text

def generate_launch_description():
    map_yaml_file = launch.substitutions.LaunchConfiguration('map')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    params_file = launch.substitutions.LaunchConfiguration('params', default=
        [launch.substitutions.ThisLaunchFileDir(), '/nav2_params.yaml'])
    final_params = RewrittenYaml(params_file, {'use_sim_time': use_sim_time })

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

class DictItemReference:
  def __init__(self, dictionary, key):
    self.dictionary = dictionary
    self.dictKey = key

  def key(self):
      return self.dictKey

  def setValue(self, value):
      self.dictionary[self.dictKey] = value

class RewrittenYaml(launch.Substitution):
  """
  Substitution that modifies the given Yaml file, given a dictionary of keys and substitutions

  The dictionary is something along the lines of {'use_sim_time': substition_value }
  and this would take every key matching 'use_sim_time' in the given Yaml file and
  replace it with the substitution value.

  The resulting yaml data is written to a temporary
  """

  def __init__(self, source_file: launch.SomeSubstitutionsType, rewrites: Dict) -> None:
    super().__init__()

    from launch.utilities import normalize_to_list_of_substitutions
    self.__source_file = normalize_to_list_of_substitutions(source_file)
    self.__rewrites = {}
    for key in rewrites:
        self.__rewrites[key] = normalize_to_list_of_substitutions(rewrites[key])

  @property
  def name(self) -> List[launch.Substitution]:
    """Getter for name."""
    return self.__source_file

  def describe(self) -> Text:
    """Return a description of this substitution as a string."""
    return 'RewrittenYaml'

  def perform(self, context: launch.LaunchContext) -> Text:
    # resolve name of input Yaml file
    original_yaml_filename = launch.utilities.perform_substitutions(context, self.name)
    # create temporary file for output
    rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
    # resolve all substitions in the rewrites dict
    resolved_rewrites = self.resolve_rewrites(context)
    data = yaml.load(open(original_yaml_filename, 'r'))
    self.substitute_values(data, resolved_rewrites)
    yaml.dump(data, rewritten_yaml)
    rewritten_yaml.close()
    return rewritten_yaml.name

  # The rewrite dictionary consists of key - value pairs. The key is a plain
  # string. The value is a launch.substitution. This function resolves the
  # substitions.
  def resolve_rewrites(self, context):
    resolved = {}
    for key in self.__rewrites:
      resolved[key] = launch.utilities.perform_substitutions(context, self.__rewrites[key])
    return resolved

  # Modifies the input yaml data with the values specified in rewrite dictionary
  def substitute_values(self, yaml, rewrites):
    for key in self.getYamlKeys(yaml):
      if key.key() in rewrites:
        key.setValue(rewrites[key.key()])

  # create a generator to allow iterating through all keys in the yaml file
  # only iterates through dictionaries. Doesn't iterate through lists.
  def getYamlKeys(self, yamlData):
    try:
      for key in yamlData.keys():
        for k in self.getYamlKeys(yamlData[key]):
          yield k
        yield DictItemReference(yamlData, key)
    except AttributeError:
      return
