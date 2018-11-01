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

"""Module for the PythonExpression substitution."""

import collections
from typing import List
from typing import Text

from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import ensure_argument_type


class PythonExpression(Substitution):
    """
    Substitution that can access contextual local variables.

    The expression may contain Substitutions, but must return something that can
    be converted to a string with `str()`.
    """

    def __init__(self, expression: SomeSubstitutionsType) -> None:
        """Constructor."""
        super().__init__()

        ensure_argument_type(
            expression,
            (str, Substitution, collections.Iterable),
            'expression',
            'PythonExpression')

        from ..utilities import normalize_to_list_of_substitutions
        self.__expression = normalize_to_list_of_substitutions(expression)

    @property
    def expression(self) -> List[Substitution]:
        """Getter for expression."""
        return self.__expression

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'PythonExpr({})'.format(' + '.join([sub.describe() for sub in self.expression]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by evaluating the expression."""
        from ..utilities import perform_substitutions
        return str(eval(perform_substitutions(context, self.expression)))
