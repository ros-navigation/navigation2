from typing import Dict
from typing import List
from typing import Text
import yaml
import tempfile
import launch

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
  Substitution that modifies the given Yaml file.
  """

  def __init__(self, source_file: launch.SomeSubstitutionsType, rewrites: Dict) -> None:
    super().__init__()

    from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
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
    resolved = {}
    for key in self.__rewrites:
      resolved[key] = launch.utilities.perform_substitutions(context, self.__rewrites[key])
    return resolved

  def substitute_values(self, yaml, rewrites):
    for key in self.getYamlKeys(yaml):
      if key.key() in rewrites:
        key.setValue(rewrites[key.key()])

  def getYamlKeys(self, yamlData):
    try:
      for key in yamlData.keys():
        for k in self.getYamlKeys(yamlData[key]):
          yield k
        yield DictItemReference(yamlData, key)
    except AttributeError:
      return
