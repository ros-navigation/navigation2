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

"""Module for LaunchContext class."""

import asyncio
import collections
import logging
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List  # noqa: F401
from typing import Optional
from typing import Text

from .event import Event
from .event_handler import EventHandler
from .substitution import Substitution

_logger = logging.getLogger(name='launch')


class LaunchContext:
    """Runtime context used by various launch entities when being visited or executed."""

    def __init__(self, *, argv: Optional[Iterable[Text]] = None) -> None:
        """
        Constructor.

        :param: argv stored in the context for access by the entities, None results in []
        """
        self.__argv = argv if argv is not None else []

        self._event_queue = asyncio.Queue()  # type: asyncio.Queue
        self._event_handlers = collections.deque()  # type: collections.deque
        self._completion_futures = []  # type: List[asyncio.Future]

        self.__globals = {}  # type: Dict[Text, Any]
        self.__locals_stack = []  # type: List[Dict[Text, Any]]
        self.__locals = {}  # type: Dict[Text, Any]
        self.__combined_locals_cache = None  # type: Optional[Dict[Text, Any]]

        self.__launch_configurations_stack = []  # type: List[Dict[Text, Text]]
        self.__launch_configurations = {}  # type: Dict[Text, Text]

        self.__is_shutdown = False
        self.__asyncio_loop = None  # type: Optional[asyncio.AbstractEventLoop]

    @property
    def argv(self):
        """Getter for argv."""
        return self.__argv

    def _set_is_shutdown(self, state: bool) -> None:
        self.__is_shutdown = state

    @property
    def is_shutdown(self):
        """Getter for is_shutdown."""
        return self.__is_shutdown

    def _set_asyncio_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self.__asyncio_loop = loop

    @property
    def asyncio_loop(self):
        """Getter for asyncio_loop."""
        return self.__asyncio_loop

    def add_completion_future(self, completion_future: asyncio.Future) -> None:
        """Add an asyncio.Future to the list of futures that the LaunchService will wait on."""
        self._completion_futures.append(completion_future)

    def _push_locals(self):
        self.__locals_stack.append(dict(self.__locals))

    def _pop_locals(self):
        if not self.__locals_stack:
            raise RuntimeError('locals stack unexpectedly empty')
        self.__locals = self.__locals_stack.pop()
        self._clear_combined_locals_cache()

    def extend_globals(self, extensions: Dict[Text, Any]) -> None:
        """
        Extend the context.locals object permanently with new members.

        These "globals" are in the context.locals, but are overridden by
        overlapping keys provided by extend_locals().
        """
        self.__globals.update(extensions)
        self._clear_combined_locals_cache()

    def extend_locals(self, extensions: Dict[Text, Any]) -> None:
        """Extend the context.locals object with new members until popped."""
        self.__locals.update(extensions)
        self._clear_combined_locals_cache()

    def _clear_combined_locals_cache(self):
        self.__combined_locals_cache = None

    def _get_combined_locals(self):
        if self.__combined_locals_cache is None:
            self.__combined_locals_cache = dict(self.__globals)
            self.__combined_locals_cache.update(self.__locals)
        return self.__combined_locals_cache

    def get_locals_as_dict(self) -> Dict[Text, Any]:
        """Access the context locals as a dictionary."""
        return self._get_combined_locals()

    @property  # noqa: A003
    def locals(self):
        """Getter for the locals."""
        class AttributeDict:

            def __init__(self, dict_in):
                self.__dict__['__dict'] = dict_in

            def __getattr__(self, key):
                _dict = self.__dict__['__dict']
                if key not in _dict:
                    raise AttributeError(
                        "context.locals does not contain attribute '{}', it contains: [{}]".format(
                            key,
                            ', '.join(_dict.keys())
                        )
                    )
                return _dict[key]

            def __setattr__(self, key, value):
                raise AttributeError("can't set attribute '{}', locals are read-only".format(key))

        return AttributeDict(self._get_combined_locals())

    def _push_launch_configurations(self):
        self.__launch_configurations_stack.append(dict(self.__launch_configurations))

    def _pop_launch_configurations(self):
        if not self.__launch_configurations_stack:
            raise RuntimeError('launch_configurations stack unexpectedly empty')
        self.__launch_configurations = self.__launch_configurations_stack.pop()

    @property
    def launch_configurations(self) -> Dict[Text, Text]:
        """Getter for launch_configurations dictionary."""
        return self.__launch_configurations

    def register_event_handler(self, event_handler: EventHandler) -> None:
        """Register a event handler."""
        self._event_handlers.appendleft(event_handler)

    def unregister_event_handler(self, event_handler: EventHandler) -> None:
        """Unregister an event handler."""
        self._event_handlers.remove(event_handler)

    def emit_event_sync(self, event: Event) -> None:
        """Emit an event synchronously."""
        _logger.debug("emitting event synchronously: '{}'".format(event.name))
        self._event_queue.put_nowait(event)

    async def emit_event(self, event: Event) -> None:
        """Emit an event."""
        _logger.debug("emitting event: '{}'".format(event.name))
        await self._event_queue.put(event)

    def perform_substitution(self, substitution: Substitution) -> Text:
        """Perform substitution on given Substitution."""
        return substitution.perform(self)
