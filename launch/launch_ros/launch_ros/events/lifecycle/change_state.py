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

"""Module for ChangeState event."""

import collections
from typing import Callable

from launch.event import Event

import lifecycle_msgs.msg

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ...actions import LifecycleNode  # noqa


class ChangeState(Event):
    """Event emitted when a state transition is requested for a lifecycle node."""

    name = 'launch_ros.events.lifecycle.ChangeState'
    valid_transitions = collections.OrderedDict([
        (lifecycle_msgs.msg.Transition.TRANSITION_CREATE, 'TRANSITION_CREATE'),
        (lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE, 'TRANSITION_CONFIGURE'),
        (lifecycle_msgs.msg.Transition.TRANSITION_CLEANUP, 'TRANSITION_CLEANUP'),
        (lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE, 'TRANSITION_ACTIVATE'),
        (lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE, 'TRANSITION_DEACTIVATE'),
        (lifecycle_msgs.msg.Transition.TRANSITION_SHUTDOWN, 'TRANSITION_SHUTDOWN'),
        (lifecycle_msgs.msg.Transition.TRANSITION_DESTROY, 'TRANSITION_DESTROY'),
    ])
    valid_states = collections.OrderedDict([
        (lifecycle_msgs.msg.State.PRIMARY_STATE_UNKNOWN, 'PRIMARY_STATE_UNKNOWN'),
        (lifecycle_msgs.msg.State.PRIMARY_STATE_UNCONFIGURED, 'PRIMARY_STATE_UNCONFIGURED'),
        (lifecycle_msgs.msg.State.PRIMARY_STATE_INACTIVE, 'PRIMARY_STATE_INACTIVE'),
        (lifecycle_msgs.msg.State.PRIMARY_STATE_ACTIVE, 'PRIMARY_STATE_ACTIVE'),
        (lifecycle_msgs.msg.State.PRIMARY_STATE_FINALIZED, 'PRIMARY_STATE_FINALIZED'),
        (lifecycle_msgs.msg.State.TRANSITION_STATE_CONFIGURING, 'TRANSITION_STATE_CONFIGURING'),
        (lifecycle_msgs.msg.State.TRANSITION_STATE_CLEANINGUP, 'TRANSITION_STATE_CLEANINGUP'),
        (lifecycle_msgs.msg.State.TRANSITION_STATE_SHUTTINGDOWN, 'TRANSITION_STATE_SHUTTINGDOWN'),
        (lifecycle_msgs.msg.State.TRANSITION_STATE_ACTIVATING, 'TRANSITION_STATE_ACTIVATING'),
        (lifecycle_msgs.msg.State.TRANSITION_STATE_DEACTIVATING, 'TRANSITION_STATE_DEACTIVATING'),
        (
            lifecycle_msgs.msg.State.TRANSITION_STATE_ERRORPROCESSING,
            'TRANSITION_STATE_ERRORPROCESSING',
        ),
    ])

    def __init__(
        self,
        *,
        lifecycle_node_matcher: Callable[['LifecycleNode'], bool],
        transition_id: int
    ) -> None:
        """
        Constructor.

        :param: lifecycle_node_matcher is a callable which returns True if the
            given lifecycle node should be affected by this event.
        :param: transition_id is the id of the requested transition which are
            defined in the :class:`lifecycle_msgs.msg.Transition` message class,
            e.g. `lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE`.
        """
        super().__init__()
        self.__lifecycle_node_matcher = lifecycle_node_matcher
        self.__transition_id = transition_id
        if transition_id not in self.valid_transitions.keys():
            raise ValueError("given transition_id of '{}', expected one of {{{}}}".format(
                transition_id,
                ', '.join(['{}: {}'.format(v, k) for k, v in self.valid_transitions.items()]),
            ))

    @property
    def lifecycle_node_matcher(self) -> Callable[['LifecycleNode'], bool]:
        """Getter for lifecycle_node_matcher."""
        return self.__lifecycle_node_matcher

    @property
    def transition_id(self) -> int:
        """Getter for transition_id."""
        return self.__transition_id
