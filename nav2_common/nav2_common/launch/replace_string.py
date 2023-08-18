# Copyright (c) 2019 Intel Corporation
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

from typing import Dict
from typing import List
from typing import Text
from typing import Optional
import tempfile
import launch

class ReplaceString(launch.Substitution):
  """
  Substitution that replaces strings on a given file.

  Used in launch system
  """

  def __init__(self,
    source_file: launch.SomeSubstitutionsType,
    replacements: Dict,
    condition: Optional[launch.Condition] = None) -> None:
    super().__init__()

    from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
    self.__source_file = normalize_to_list_of_substitutions(source_file)
    self.__replacements = {}
    for key in replacements:
        self.__replacements[key] = normalize_to_list_of_substitutions(replacements[key])
    self.__condition = condition

  @property
  def name(self) -> List[launch.Substitution]:
    """Getter for name."""
    return self.__source_file

  @property
  def condition(self) -> Optional[launch.Condition]:
    """Getter for condition."""
    return self.__condition

  def describe(self) -> Text:
    """Return a description of this substitution as a string."""
    return ''

  def perform(self, context: launch.LaunchContext) -> Text:
    yaml_filename = launch.utilities.perform_substitutions(context, self.name)
    if self.__condition is None or self.__condition.evaluate(context):
      output_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
      replacements = self.resolve_replacements(context)
      try:
        input_file = open(yaml_filename, 'r')
        self.replace(input_file, output_file, replacements)
      except Exception as err:  # noqa: B902
        print('ReplaceString substitution error: ', err)
      finally:
        input_file.close()
        output_file.close()
      return output_file.name
    else:
      return yaml_filename

  def resolve_replacements(self, context):
    resolved_replacements = {}
    for key in self.__replacements:
      resolved_replacements[key] = launch.utilities.perform_substitutions(context, self.__replacements[key])
    return resolved_replacements

  def replace(self, input_file, output_file, replacements):
    for line in input_file:
      for key, value in replacements.items():
        if isinstance(key, str) and isinstance(value, str):
          if key in line:
            line = line.replace(key, value)
        else:
          raise TypeError('A provided replacement pair is not a string. Both key and value should be strings.')
      output_file.write(line)
