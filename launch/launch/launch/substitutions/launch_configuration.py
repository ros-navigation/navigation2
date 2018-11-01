# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the LaunchConfiguration substitution."""

import collections
from typing import Any
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Union

from .substitution_failure import SubstitutionFailure
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import ensure_argument_type


class LaunchConfiguration(Substitution):
    """Substitution that can access launch configuration variables."""

    def __init__(
        self,
        variable_name: SomeSubstitutionsType,
        *,
        default: Optional[Union[Any, Iterable[Any]]] = None
    ) -> None:
        """Constructor."""
        super().__init__()

        ensure_argument_type(variable_name, str, 'variable_name', 'LaunchConfiguration')

        from ..utilities import normalize_to_list_of_substitutions
        self.__variable_name = normalize_to_list_of_substitutions(variable_name)
        if default is None:
            self.__default = default
        else:
            # convert any items in default that are not a Substitution or str to a str
            str_normalized_default = []  # type: List[Union[Text, Substitution]]
            definitely_iterable_default = ((),)  # type: Iterable[Any]
            if isinstance(default, collections.Iterable):
                definitely_iterable_default = default
            else:
                definitely_iterable_default = (default,)
            for item in definitely_iterable_default:
                if isinstance(item, (str, Substitution)):
                    str_normalized_default.append(item)
                else:
                    str_normalized_default.append(str(item))
            # use normalize_to_list_of_substitutions to convert str to TextSubstitution's too
            self.__default = \
                normalize_to_list_of_substitutions(
                    str_normalized_default)  # type: List[Substitution]

    @property
    def variable_name(self) -> List[Substitution]:
        """Getter for variable_name."""
        return self.__variable_name

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'LaunchConfig({})'.format(' + '.join([s.describe() for s in self.variable_name]))

    def perform(self, context: LaunchContext) -> Text:
        """
        Perform the substitution by retrieving the launch configuration, as a string.

        If the launch configuration is not found and a default has been set,
        the default will be returned, as a string.
        """
        from ..utilities import perform_substitutions
        expanded_variable_name = perform_substitutions(context, self.__variable_name)
        if expanded_variable_name not in context.launch_configurations:
            if self.__default is None:
                raise SubstitutionFailure(
                    "launch configuration '{}' does not exist".format(expanded_variable_name))
            else:
                return perform_substitutions(context, self.__default)
        return context.launch_configurations[expanded_variable_name]
