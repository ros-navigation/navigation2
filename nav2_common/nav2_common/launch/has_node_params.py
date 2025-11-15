# Copyright (c) 2021 PAL Robotics S.L.
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

from typing import Sequence

import launch
import yaml


@launch.frontend.expose_substitution('has-node-params')
class HasNodeParams(launch.Substitution):
    """
    Substitution that checks if a param file contains parameters for a node.

    Used in launch system
    """

    def __init__(
        self, source_file: launch.SomeSubstitutionsType, node_name: str
    ) -> None:
        super().__init__()
        """
    Construct the substitution

    :param: source_file the parameter YAML file
    :param: node_name the name of the node to check
    """

        # import here to avoid loop
        from launch.utilities import normalize_to_list_of_substitutions

        self.__source_file: list[launch.Substitution] = \
            normalize_to_list_of_substitutions(source_file)
        self.__node_name = node_name

    @classmethod
    def parse(cls, data: Sequence[launch.SomeSubstitutionsType]):
        num_args = len(data)
        if num_args != 2:
            raise TypeError(
                f'has-node-params substitution takes 2 arguments, got {num_args}')
        kwargs = {
            'source_file': data[0],
            'node_name': data[1],
        }
        return cls, kwargs

    @property
    def name(self) -> list[launch.Substitution]:
        """Getter for name."""
        return self.__source_file

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: launch.LaunchContext) -> str:
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        data = yaml.safe_load(open(yaml_filename))

        if self.__node_name in data.keys():
            return 'True'
        return 'False'
