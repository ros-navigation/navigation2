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

"""Module for the IncludeLaunchDescription action."""

import os
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple

from .set_launch_configuration import SetLaunchConfiguration
from ..action import Action
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..launch_description_source import LaunchDescriptionSource
from ..some_substitutions_type import SomeSubstitutionsType
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


class IncludeLaunchDescription(Action):
    """
    Action that includes a launch description source and yields its entities when visited.

    It is possible to pass arguments to the launch description, which it
    declared with the :py:class:`launch.actions.DeclareLaunchArgument` action.

    If any given arguments do not match the name of any declared launch
    arguments, then they will still be set as Launch Configurations using the
    :py:class:`launch.actions.SetLaunchConfiguration` action.
    This is done because it's not always possible to detect all instances of
    the declare launch argument class in the given launch description.

    On the other side, an error will sometimes be raised if the given launch
    description declares a launch argument and its value is not provided to
    this action.
    It will only produce this error, however, if the declared launch argument
    is unconditional (sometimes the action that declares the launch argument
    will only be visited in certain circumstances) and if it does not have a
    default value on which to fall back.

    Conditionally included launch arguments that do not have a default value
    will eventually raise an error if this best effort argument checking is
    unable to see an unsatisfied argument ahead of time.
    """

    def __init__(
        self,
        launch_description_source: LaunchDescriptionSource,
        *,
        launch_arguments: Optional[
            Iterable[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]
        ] = None
    ) -> None:
        """Constructor."""
        super().__init__()
        self.__launch_description_source = launch_description_source
        self.__launch_arguments = launch_arguments

    @property
    def launch_description_source(self) -> LaunchDescriptionSource:
        """Getter for self.__launch_description_source."""
        return self.__launch_description_source

    @property
    def launch_arguments(self) -> Iterable[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]:
        """Getter for self.__launch_arguments."""
        if self.__launch_arguments is None:
            return []
        else:
            return self.__launch_arguments

    def _get_launch_file_location(self):
        launch_file_location = os.path.abspath(self.__launch_description_source.location)
        if os.path.exists(launch_file_location):
            launch_file_location = os.path.dirname(launch_file_location)
        else:
            # If the location does not exist, then it's likely set to '<script>' or something
            # so just pass it along.
            launch_file_location = self.__launch_description_source.location
        return launch_file_location

    def describe_sub_entities(self) -> List[LaunchDescriptionEntity]:
        """Override describe_sub_entities from LaunchDescriptionEntity to return sub entities."""
        ret = self.__launch_description_source.try_get_launch_description_without_context()
        return [ret] if ret is not None else []

    def visit(self, context: LaunchContext) -> List[LaunchDescriptionEntity]:
        """Override visit to return an Entity rather than an action."""
        launch_description = self.__launch_description_source.get_launch_description(context)
        context.extend_locals({
            'current_launch_file_directory': self._get_launch_file_location(),
        })

        # Do best effort checking to see if non-optional, non-default declared arguments
        # are being satisfied.
        argument_names = [
            perform_substitutions(context, normalize_to_list_of_substitutions(arg_name))
            for arg_name, arg_value in self.launch_arguments
        ]
        declared_launch_arguments = launch_description.get_launch_arguments()
        for argument in declared_launch_arguments:
            if argument._conditionally_included or argument.default_value is not None:
                continue
            if argument.name not in argument_names:
                raise RuntimeError(
                    "Included launch description missing required argument '{}' "
                    "(description: '{}'), given: [{}]"
                    .format(argument.name, argument.description, ', '.join(argument_names))
                )

        # Create actions to set the launch arguments into the launch configurations.
        set_launch_configuration_actions = []
        for name, value in self.launch_arguments:
            set_launch_configuration_actions.append(SetLaunchConfiguration(name, value))

        # Set launch arguments as launch configurations and then include the launch description.
        return [*set_launch_configuration_actions, launch_description]
