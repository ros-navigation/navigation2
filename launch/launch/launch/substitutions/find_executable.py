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

"""Module for the FindExecutable substitution."""

from typing import List
from typing import Text

from osrf_pycommon.process_utils import which

from .substitution_failure import SubstitutionFailure
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution


class FindExecutable(Substitution):
    """
    Substitution that tries to locate an executable on the PATH.

    :raise: SubstitutionFailure when executable not found
    """

    def __init__(self, *, name: SomeSubstitutionsType) -> None:
        """Constructor."""
        super().__init__()

        from ..utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self.__name = normalize_to_list_of_substitutions(name)

    @property
    def name(self) -> List[Substitution]:
        """Getter for name."""
        return self.__name

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'FindExec({})'.format(' + '.join([sub.describe() for sub in self.name]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by locating the executable on the PATH."""
        from ..utilities import perform_substitutions  # import here to avoid loop
        result = which(perform_substitutions(context, self.name))
        if result is None:
            raise SubstitutionFailure("executable '{}' not found on the PATH".format(self.name))
        return result
