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

"""Module for LaunchDescriptionEntity class."""

import asyncio
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from .launch_context import LaunchContext  # noqa


class LaunchDescriptionEntity:
    """
    Single item in a launch description.

    These entities are "visited" during different activities like "launching"
    or "introspection".
    In each case, different methods of a LaunchDescriptionEntity may be called
    and those methods can be overridden and enhanced by derived classes.

    When being introspected, each LaunchDescriptionEntity has its describe(),
    describe_sub_entities(), and describe_conditional_sub_entities() methods
    invoked in that order.
    These methods are given no contextual information, but are allowed to
    return a description in each case.
    See the methods for more information.

    When being launched, each LaunchDescriptionEntity has its visit() method
    invoked.
    This method is given a LaunchContext and may optionally return a list of
    sub-entities which should also be immediately visited.
    After being visited, and before sub-entities or other entities are visited,
    the get_asyncio_future() method is invoked.
    This method may optionally return an :class:`asyncio.Future` object which
    should be set to "done" when this entity's on-going activities are finished.
    If there is no on-going activity, then None may be returned, which is what
    the default implementation in this class does.
    """

    def describe(self) -> Text:
        """
        Return a description of this entity as a string.

        When inherited from, calling this base class's default method is not
        required, and in fact it will raise NotImplementedError.
        """
        raise NotImplementedError()

    def describe_sub_entities(self) -> List['LaunchDescriptionEntity']:
        """
        Return a list of sub-entities which need to be described as well.

        The list may be empty.

        The sub-entities in this list should always be returned when this
        entity is visited at runtime.
        If there are entities which are only returned under some condition,
        use the describe_conditional_sub_entities() method instead.

        In the case of multiple layers of inheritance, you may wish to call
        other base class's describe_sub_entities() and extend your own list of
        sub-entities depending on the behavior of your class, but calling this
        default method is not required.
        """
        return []

    def describe_conditional_sub_entities(self) -> List[Tuple[
        Text,  # text description of the condition
        Iterable['LaunchDescriptionEntity'],  # list of conditional sub-entities
    ]]:
        """
        Return a list of condition descriptions and lists of sub-entities.

        The list of conditional sub-entities are made up of two item tuples,
        where the first item is a text description of the condition, and the
        second item is a list of sub-entities which would be visited if the
        condition is evaluated to be true during launch.

        In the case of multiple layers of inheritance, you may wish to call the
        base class's describe_conditional_sub_entities() and extend your own
        list of sub-entities depending on the behavior of your class, but
        calling this default method is not required.
        """
        return []

    def visit(self, context: 'LaunchContext') -> Optional[List['LaunchDescriptionEntity']]:
        """
        Visit the entity.

        This is called for each entity in a launch description when being
        evaluated at runtime.

        Should be overridden by derived class, but by default does nothing.

        This method should not block, and should aim to be as fast as possible,
        because while this method is running other tasks and events cannot be
        handled.
        Blocking should be done asynchronously using Python's asyncio.
        The loop can be accessed from the context with the `asyncio_loop`
        member.

        Consider using the `create_task(coroutine)` method of the loop for
        asynchronous coroutines, or the `run_in_executor()` method of the loop
        for long running synchronous tasks.

        This method is called from within the loop's `run_forever()` or similar
        method, but it is not awaited by the called as it is not a coroutine
        itself.

        If you have on-going asynchronous tasks, also override the
        :meth:`get_asyncio_future()` and use the future you return from it to
        let the launch system know when your entity is done with all pending
        work.
        As an example, an entity which ran a subprocess might wait until that
        subprocess exits before setting the future as "done" so that the
        launch system doesn't exit until all child processes are joined.
        """
        return None

    def get_asyncio_future(self) -> Optional[asyncio.Future]:
        """
        Return an asyncio Future, or None if there are no on-going tasks.

        This method is always called after visit().

        The returned future should be completed when there is no on-going
        asynchronous tasks associated with the entity.

        This is used by the launch system to know when it is idle, i.e. when
        all entities have not only been visited, but also have been "completed".

        If the launch system is trying to shutdown, and this entity is not
        setting the returned future to done quickly enough, the launch system
        may call :meth:`asyncio.Future.cancel()`, which will cause anything
        awaiting this future to raise the :exc:`CancelledError` exception.
        """
        return None
