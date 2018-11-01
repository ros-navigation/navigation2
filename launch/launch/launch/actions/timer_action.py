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

"""Module for the TimerAction action."""

import asyncio
import collections
import logging
from typing import Any  # noqa: F401
from typing import cast
from typing import Dict  # noqa: F401
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import Union

from ..action import Action
from ..event_handler import EventHandler
from ..events import TimerEvent
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_actions_type import SomeActionsType
from ..some_substitutions_type import SomeSubstitutionsType
from ..some_substitutions_type import SomeSubstitutionsType_types_tuple
from ..utilities import create_future
from ..utilities import ensure_argument_type
from ..utilities import is_a_subclass
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions

_event_handler_has_been_installed = False
_logger = logging.getLogger('launch.timer_action')


class TimerAction(Action):
    """
    Action that defers other entities until a period of time has passed, unless canceled.

    All timers are "one-shot", in that they only fire one time and never again.
    """

    def __init__(
        self,
        *,
        period: Union[float, SomeSubstitutionsType],
        actions: Iterable[LaunchDescriptionEntity],
        **kwargs
    ) -> None:
        """Constructor."""
        super().__init__(**kwargs)
        period_types = list(SomeSubstitutionsType_types_tuple) + [float]
        ensure_argument_type(period, period_types, 'period', 'TimerAction')
        ensure_argument_type(actions, collections.Iterable, 'actions', 'TimerAction')
        if isinstance(period, float):
            self.__period = normalize_to_list_of_substitutions([str(period)])
        else:
            self.__period = normalize_to_list_of_substitutions(period)
        self.__actions = actions
        self.__context_locals = {}  # type: Dict[Text, Any]
        self.__completed_future = None  # type: Optional[asyncio.Future]
        self.__canceled = False
        self.__canceled_future = None  # type: Optional[asyncio.Future]

    async def __wait_to_fire_event(self, context):
        done, pending = await asyncio.wait(
            [self.__canceled_future],
            loop=context.asyncio_loop,
            timeout=float(perform_substitutions(context, self.__period)),
        )
        if not self.__canceled_future.done():
            await context.emit_event(TimerEvent(timer_action=self))
        self.__completed_future.set_result(None)

    def describe(self) -> Text:
        """Return a description of this TimerAction."""
        return 'TimerAction(period={}, actions=<actions>)'.format(self.__period)

    # def describe_conditional_sub_entities(self) -> List[Tuple[
    #     Text,
    #     Iterable['LaunchDescriptionEntity'],
    # ]]:
    #     """Return the actions that will result when the timer expires, but was not canceled."""
    #     return [('{} seconds pass without being canceled'.format(self.__period), self.__actions)]

    def handle(self, context: LaunchContext) -> Optional[SomeActionsType]:
        """Handle firing of timer."""
        context.extend_locals(self.__context_locals)
        return self.__actions

    def cancel(self) -> None:
        """
        Cancel this TimerAction.

        Calling cancel will not fail if the timer has already finished or
        already been canceled or if the timer has not been started yet.

        This function is not thread-safe and should be called only from under
        another coroutine.
        """
        self.__canceled = True
        if self.__canceled_future is not None and not self.__canceled_future.done():
            self.__canceled_future.set_result(True)
        return None

    def execute(self, context: LaunchContext) -> Optional[List['Action']]:
        """
        Execute the action.

        This does the following:
        - register a global event handler for TimerAction's if not already done
        - create a task for the coroutine that waits until canceled or timeout
        - coroutine asynchronously fires event after timeout, if not canceled
        """
        self.__completed_future = create_future(context.asyncio_loop)
        self.__canceled_future = create_future(context.asyncio_loop)

        if self.__canceled:
            # In this case, the action was canceled before being executed.
            _logger.debug(
                'timer {} not waiting because it was canceled before being executed'.format(self)
            )
            self.__completed_future.set_result(None)
            return None

        # Once globally, install the general purpose OnTimerEvent event handler.
        global _event_handler_has_been_installed
        if not _event_handler_has_been_installed:
            from ..actions import OpaqueFunction
            context.register_event_handler(EventHandler(
                matcher=lambda event: is_a_subclass(event, TimerEvent),
                entities=OpaqueFunction(
                    function=lambda context: (
                        cast(TimerEvent, context.locals.event).timer_action.handle(context)
                    )
                ),
            ))
            _event_handler_has_been_installed = True

        # Capture the current context locals so the yielded actions can make use of them too.
        self.__context_locals = dict(context.get_locals_as_dict())  # Capture a copy
        context.asyncio_loop.create_task(self.__wait_to_fire_event(context))
        return None

    def get_asyncio_future(self) -> Optional[asyncio.Future]:
        """Return an asyncio Future, used to let the launch system know when we're done."""
        return self.__completed_future
