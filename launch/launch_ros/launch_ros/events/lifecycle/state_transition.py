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

"""Module for StateTransition event."""

from typing import Text

from launch.event import Event

import lifecycle_msgs.msg

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ...actions import LifecycleNode  # noqa


class StateTransition(Event):
    """Event emitted when a lifecycle node makes a state transition."""

    name = 'launch_ros.events.lifecycle.StateTransition'

    def __init__(
        self,
        *,
        action: 'LifecycleNode',
        msg: lifecycle_msgs.msg.TransitionEvent
    ) -> None:
        """
        Constructor.

        :param: action the instance of class::`LifecycleNode` that generated this event
        :param: msg the instance of the ROS message TransitionEvent that generated this event
        """
        super().__init__()
        self.__action = action
        self.__msg = msg
        self.__timestamp = msg.timestamp
        self.__transition = msg.transition.label
        self.__start_state = msg.start_state.label
        self.__goal_state = msg.goal_state.label

    @property
    def action(self) -> 'LifecycleNode':
        """Getter for action."""
        return self.__action

    @property
    def msg(self) -> lifecycle_msgs.msg.TransitionEvent:
        """Getter for msg."""
        return self.__msg

    @property
    def timestamp(self) -> int:
        """Getter for timestamp."""
        return self.__timestamp

    @property
    def transition(self) -> Text:
        """Getter for transition."""
        return self.__transition

    @property
    def start_state(self) -> Text:
        """Getter for start_state."""
        return self.__start_state

    @property
    def goal_state(self) -> Text:
        """Getter for goal_state."""
        return self.__goal_state
