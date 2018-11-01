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

"""Module for IfCondition class."""

from typing import Text

from .evaluate_condition_expression_impl import evaluate_condition_expression
from ..condition import Condition
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..utilities import normalize_to_list_of_substitutions


class IfCondition(Condition):
    """
    Encapsulates an if condition to be evaluated when launching.

    This condition takes a string expression that is lexically evaluated as a
    boolean, but the expression may consist of :py:class:`launch.Substitution`
    instances.

    See :py:func:`evaluate_condition_expression` to understand what constitutes
    a valid condition expression.
    """

    def __init__(self, predicate_expression: SomeSubstitutionsType) -> None:
        self.__predicate_expression = normalize_to_list_of_substitutions(predicate_expression)
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context: LaunchContext) -> bool:
        return evaluate_condition_expression(context, self.__predicate_expression)

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()
