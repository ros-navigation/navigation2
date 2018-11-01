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

"""Module for OnProcessExit class."""

import collections
from typing import Callable
from typing import cast
from typing import Optional
from typing import overload
from typing import Text

from ..event import Event
from ..event_handler import EventHandler
from ..events.process import ProcessExited
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_actions_type import SomeActionsType


if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ..actions import ExecuteProcess  # noqa


class OnProcessExit(EventHandler):
    """
    Convenience class for handling a process exited event.

    It may be configured to only handle the exiting of a specific action,
    or to handle all exited processes.
    """

    @overload
    def __init__(
        self, *,
        target_action: 'ExecuteProcess' = None,
        on_exit: SomeActionsType,
        **kwargs
    ) -> None:
        """Overload which takes just actions."""
        ...

    @overload  # noqa: F811
    def __init__(
        self,
        *,
        target_action: 'ExecuteProcess' = None,
        on_exit: Callable[[int], Optional[SomeActionsType]],
        **kwargs
    ) -> None:
        """Overload which takes a callable to handle the exit."""
        ...

    def __init__(self, *, target_action=None, on_exit, **kwargs) -> None:  # noqa: F811
        """Constructor."""
        from ..actions import ExecuteProcess  # noqa
        if not isinstance(target_action, (ExecuteProcess, type(None))):
            raise RuntimeError("OnProcessExit requires an 'ExecuteProcess' action as the target")
        super().__init__(
            matcher=(
                lambda event: (
                    isinstance(event, ProcessExited) and (
                        target_action is None or
                        event.action == target_action
                    )
                )
            ),
            entities=None,
            **kwargs,
        )
        self.__target_action = target_action
        # TODO(wjwwood) check that it is not only callable, but also a callable that matches
        # the correct signature for a handler in this case
        self.__on_exit = on_exit
        self.__actions_on_exit = []  # type: List[LaunchDescriptionEntity]
        if callable(on_exit):
            # Then on_exit is a function or lambda, so we can just call it, but
            # we don't put anything in self.__actions_on_exit because we cannot
            # know what the function will return.
            pass
        else:
            # Otherwise, setup self.__actions_on_exit
            if isinstance(on_exit, collections.Iterable):
                for entity in on_exit:
                    if not isinstance(entity, LaunchDescriptionEntity):
                        raise ValueError(
                            "expected all items in 'on_exit' iterable to be of type "
                            "'LaunchDescriptionEntity' but got '{}'".format(type(entity)))
                self.__actions_on_exit = list(on_exit)  # Outside list is to ensure type is List
            else:
                self.__actions_on_exit = [on_exit]
            # Then return it from a lambda and use that as the self.__on_exit callback.
            self.__on_exit = lambda event, context: self.__actions_on_exit

    def handle(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        """Handle the given event."""
        return self.__on_exit(cast(ProcessExited, event), context)

    @property
    def handler_description(self) -> Text:
        """Return the string description of the handler."""
        # TODO(jacobperron): revisit how to describe known actions that are passed in.
        #                    It would be nice if the parent class could output their description
        #                    via the 'entities' property.
        if self.__actions_on_exit:
            return '<actions>'
        return '{}'.format(self.__on_exit)

    @property
    def matcher_description(self) -> Text:
        """Return the string description of the matcher."""
        if self.__target_action is None:
            return 'event == ProcessExited'
        return 'event == ProcessExited and event.action == ExecuteProcess({})'.format(
            hex(id(self.__target_action))
        )
