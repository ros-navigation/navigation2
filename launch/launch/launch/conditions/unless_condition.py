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

"""Module for UnlessCondition class."""

from typing import Text

from .if_condition import IfCondition
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType


class UnlessCondition(IfCondition):
    """
    Encapsulates an if condition to be evaluated when launching.

    Exactly the same as :py:class:`IfCondition` except it returns true if the
    condition expression evaluates to false.
    """

    def __init__(self, predicate_expression: SomeSubstitutionsType) -> None:
        super().__init__(predicate_expression)

    def _predicate_func(self, context: LaunchContext) -> bool:
        return not super()._predicate_func(context)

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()
