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

"""Module for the DeclareLaunchArgument action."""

import logging
from typing import List
from typing import Optional
from typing import Text

from ..action import Action
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions

_logger = logging.getLogger('launch.actions.DeclareLaunchArgument')


class DeclareLaunchArgument(Action):
    """
    Action that declares a new launch argument.

    A launch arguments are stored in a "launch configuration" of the same name.
    See :py:class:`launch.actions.SetLaunchConfiguration` and
    :py:class:`launch.substitutions.LaunchConfiguration`.

    Any launch arguments declared within a :py:class:`launch.LaunchDescription`
    will be exposed as arguments when that launch description is included, e.g.
    as additional parameters in the
    :py:class:`launch.actions.IncludeLaunchDescription` action or as
    command-line arguments when launched with ``ros2 launch ...``.

    In addition to the name, which is also where the argument result is stored,
    launch arguments may have a default value and a description.
    If a default value is given, then the argument becomes optional and the
    default value is placed in the launch configuration instead.
    If no default value is given and no value is given when including the
    launch description, then an error occurs.

    The default value may use Substitutions, but the name and description can
    only be Text, since they need a meaningful value before launching, e.g.
    when listing the command-line arguments.

    Note that declaring a launch argument needs to be in a part of the launch
    description that is describable without launching.
    For example, if you declare a launch argument with this action from within
    a condition group or as a callback to an event handler, then it may not be
    possible for a tool like ``ros2 launch`` to know about the argument before
    launching the launch description.
    In such cases, the argument will not be visible on the command line but
    may raise an exception if that argument is not satisfied once visited (and
    has no default value).

    Put another way, the post-condition of this action being visited is either
    that a launch configuration of the same name is set with a value or an
    exception is raised because none is set and there is no default value.
    However, the pre-condition does not guarantee that the argument was visible
    if behind condition or situational inclusions.
    """

    def __init__(
        self,
        name: Text,
        *,
        default_value: Optional[SomeSubstitutionsType] = None,
        description: Text = 'no description given',
        **kwargs
    ) -> None:
        """Constructor."""
        super().__init__(**kwargs)
        self.__name = name
        if default_value is None:
            self.__default_value = default_value
        else:
            self.__default_value = normalize_to_list_of_substitutions(default_value)
        self.__description = description

        # This is used later to determine if this launch argument will be
        # conditionally visited.
        # Its value will be read and set at different times and so the value
        # may change depending at different times based on the context.
        self._conditionally_included = False

    @property
    def name(self) -> Text:
        """Getter for self.__name."""
        return self.__name

    @property
    def default_value(self) -> Optional[List[Substitution]]:
        """Getter for self.__default_value."""
        return self.__default_value

    @property
    def description(self) -> Text:
        """Getter for self.__description."""
        return self.__description

    def execute(self, context: LaunchContext):
        """Execute the action."""
        if self.name not in context.launch_configurations:
            if self.default_value is None:
                # Argument not already set and no default value given, error.
                _logger.error(
                    "Required launch argument '{}' (description: '{}') was not provided"
                    .format(self.name, self.description)
                )
                raise RuntimeError(
                    "Required launch argument '{}' was not provided.".format(self.name))
            context.launch_configurations[self.name] = \
                perform_substitutions(context, self.default_value)
