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

"""Module for ProcessTargetedEvent event."""

from typing import Callable

from ...event import Event

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ...actions import ExecuteProcess  # noqa


class ProcessTargetedEvent(Event):
    """Event base class that is targeted at some running process."""

    name = 'launch.events.process.ProcessTargetedEvent'

    def __init__(self, *, process_matcher: Callable[['ExecuteProcess'], bool]) -> None:
        """
        Constructor.

        Some standard matchers are also available, like:

        - :func:`launch.events.process.matches_action()`
        - :func:`launch.events.process.matches_pid()`
        - :func:`launch.events.process.matches_name()`
        - :func:`launch.events.process.matches_executable()`

        :param: process_matcher is a predicate which can determine if an
            ExecuteProcess action matches this event or not
        """
        super().__init__()
        self.__process_matcher = process_matcher

    @property
    def process_matcher(self) -> Callable[['ExecuteProcess'], bool]:
        """Getter for process_matcher."""
        return self.__process_matcher
