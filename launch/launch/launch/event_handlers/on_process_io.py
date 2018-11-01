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

"""Module for OnProcessIO class."""

from typing import Callable
from typing import cast
from typing import Optional
from typing import Text

from ..event import Event
from ..event_handler import EventHandler
from ..events.process import ProcessIO
from ..launch_context import LaunchContext
from ..some_actions_type import SomeActionsType

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ..actions import ExecuteProcess  # noqa


class OnProcessIO(EventHandler):
    """Convenience class for handling I/O from processes via events."""

    # TODO(wjwwood): make the __init__ more flexible like OnProcessExit, so
    # that it can take SomeActionsType directly or a callable that returns it.
    def __init__(
        self,
        *,
        target_action: Optional['ExecuteProcess'] = None,
        on_stdin: Callable[[ProcessIO], Optional[SomeActionsType]] = None,
        on_stdout: Callable[[ProcessIO], Optional[SomeActionsType]] = None,
        on_stderr: Callable[[ProcessIO], Optional[SomeActionsType]] = None,
        **kwargs
    ) -> None:
        """Constructor."""
        from ..actions import ExecuteProcess  # noqa
        if not isinstance(target_action, (ExecuteProcess, type(None))):
            raise RuntimeError("OnProcessIO requires an 'ExecuteProcess' action as the target")
        super().__init__(matcher=self._matcher, entities=None, **kwargs)
        self.__target_action = target_action
        self.__on_stdin = on_stdin
        self.__on_stdout = on_stdout
        self.__on_stderr = on_stderr

    def _matcher(self, event: Event) -> bool:
        if not hasattr(event, '__class__'):
            raise RuntimeError("event '{}' unexpectedly not a class".format(event))
        return (
            issubclass(event.__class__, ProcessIO) and (
                self.__target_action is None or
                cast(ProcessIO, event).action == self.__target_action
            )
        )

    def handle(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        """Handle the given event."""
        event = cast(ProcessIO, event)
        if event.from_stdout and self.__on_stdout is not None:
            return self.__on_stdout(event)
        elif event.from_stderr and self.__on_stderr is not None:
            return self.__on_stderr(event)
        elif event.from_stdin and self.__on_stdin is not None:
            return self.__on_stdin(event)
        return None

    @property
    def handler_description(self) -> Text:
        """Return the string description of the handler."""
        handlers = []
        if self.__on_stdin is not None:
            handlers.append("on_stdin: '{}'".format(self.__on_stdin))
        if self.__on_stdout is not None:
            handlers.append("on_stdout: '{}'".format(self.__on_stdout))
        if self.__on_stderr is not None:
            handlers.append("on_stderr: '{}'".format(self.__on_stderr))
        handlers_str = '{' + ', '.join(handlers) + '}'
        return handlers_str

    @property
    def matcher_description(self) -> Text:
        """Return the string description of the matcher."""
        if self.__target_action is None:
            return 'event issubclass of ProcessIO'
        return 'event issubclass of ProcessIO and event.action == ExecuteProcess({})'.format(
            hex(id(self.__target_action))
        )
