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

"""Module for LaunchDescription class."""

from typing import Iterable
from typing import List
from typing import Optional

from .action import Action
from .actions import DeclareLaunchArgument
from .launch_context import LaunchContext
from .launch_description_entity import LaunchDescriptionEntity


class LaunchDescription(LaunchDescriptionEntity):
    """
    Description of a launch-able system.

    The description is expressed by a collection of entities which represent
    the system architect's intentions.

    The description may also have arguments, which are declared by
    :py:class:`launch.actions.DeclareLaunchArgument` actions within this
    launch description.

    Arguments for this description may be accessed via the
    :py:meth:`get_launch_arguments` method.
    The arguments are gathered by searching through the entities in this
    launch description and the descriptions of each entity (which may include
    entities yielded by those entities).
    """

    def __init__(
        self,
        initial_entities: Optional[Iterable[LaunchDescriptionEntity]] = None,
    ) -> None:
        """Constructor."""
        self.__entities = list(initial_entities) if initial_entities is not None else []

    def visit(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Override visit from LaunchDescriptionEntity to visit contained entities."""
        return self.__entities

    def describe_sub_entities(self) -> List[LaunchDescriptionEntity]:
        """Override describe_sub_entities from LaunchDescriptionEntity to return sub entities."""
        return self.__entities

    def get_launch_arguments(self, conditional_inclusion=False) -> List[DeclareLaunchArgument]:
        """
        Return a list of :py:class:`launch.actions.DeclareLaunchArgument` actions.

        This list is generated (never cached) by searching through this launch
        description for any instances of the action that declares launch
        arguments.

        It will use :py:meth:`launch.LaunchDescriptionEntity.describe_sub_entities`
        and :py:meth:`launch.LaunchDescriptionEntity.describe_conditional_sub_entities`
        in order to discover as many instances of the declare launch argument
        actions as is possible.
        Also, specifically in the case of the
        :py:class:`launch.actions.IncludeLaunchDescription` action, the method
        :py:meth:`launch.LaunchDescriptionSource.try_get_launch_description_without_context`
        is used to attempt to load launch descriptions without the "runtime"
        context available.
        This function may fail, e.g. if the path to the launch file to include
        uses the values of launch configurations that have not been set yet,
        and in that case the failure is ignored and the arugments defined in
        those launch files will not be seen either.

        Duplicate declarations of an argument are ignored, therefore the
        default value and description from the first instance of the argument
        declaration is used.
        """
        declared_launch_arguments = []  # type: List[DeclareLaunchArgument]

        def process_entities(entities, *, _conditional_inclusion):
            for entity in entities:
                if isinstance(entity, DeclareLaunchArgument):
                    # Avoid duplicate entries with the same name.
                    if entity.name in [e.name for e in declared_launch_arguments]:
                        continue
                    # Stuff this contextual information into the class for
                    # potential use in command-line descriptions or errors.
                    entity._conditionally_included = _conditional_inclusion
                    entity._conditionally_included |= entity.condition is not None
                    declared_launch_arguments.append(entity)
                else:
                    process_entities(
                        entity.describe_sub_entities(), _conditional_inclusion=False)
                    process_entities(
                        entity.describe_conditional_sub_entities(), _conditional_inclusion=True)

        process_entities(self.entities, _conditional_inclusion=conditional_inclusion)

        return declared_launch_arguments

    @property
    def entities(self) -> List[LaunchDescriptionEntity]:
        """Getter for the entities."""
        return self.__entities

    def add_entity(self, entity: LaunchDescriptionEntity) -> None:
        """Add an entity to the LaunchDescription."""
        self.__entities.append(entity)

    def add_action(self, action: Action) -> None:
        """Add an action to the LaunchDescription."""
        self.add_entity(action)
