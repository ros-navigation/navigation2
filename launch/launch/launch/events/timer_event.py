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

"""Module for TimerEvent event."""

from ..event import Event

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ...actions import TimerAction  # noqa


class TimerEvent(Event):
    """Event that fires when a TimerAction's period expires."""

    name = 'launch.events.TimerEvent'

    def __init__(self, *, timer_action: 'TimerAction') -> None:
        """Constructor."""
        self.__timer_action = timer_action

    @property
    def timer_action(self):
        """Getter for timer_action."""
        return self.__timer_action
