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

"""Module for Action class."""

from typing import cast
from typing import List
from typing import Optional
from typing import Text

from .condition import Condition
from .launch_context import LaunchContext
from .launch_description_entity import LaunchDescriptionEntity


class Action(LaunchDescriptionEntity):
    """
    LaunchDescriptionEntity that represents a user intention to do something.

    The action describes the intention to do something, but also can be
    executed given a :class:`launch.LaunchContext` at runtime.
    """

    def __init__(self, *, condition: Optional[Condition] = None) -> None:
        """
        Constructor.

        If the conditions argument is not None, the condition object will be
        evaluated while being visited and the action will only be executed if
        the condition evaluates to True.

        :param condition: Either a :py:class:`Condition` or None
        """
        self.__condition = condition

    @property
    def condition(self) -> Optional[Condition]:
        """Getter for condition."""
        return self.__condition

    def describe(self) -> Text:
        """Return a description of this Action."""
        return self.__repr__()

    def visit(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Override visit from LaunchDescriptionEntity so that it executes."""
        if self.__condition is None or self.__condition.evaluate(context):
            return cast(Optional[List[LaunchDescriptionEntity]], self.execute(context))
        return None

    def execute(self, context: LaunchContext) -> Optional[List['Action']]:
        """
        Execute the action.

        Should be overridden by derived class, but by default does nothing.
        """
        pass
