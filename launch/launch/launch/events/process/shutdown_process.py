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

"""Module for ShutdownProcess event."""

from typing import Callable

from .process_targeted_event import ProcessTargetedEvent

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ...actions import ExecuteProcess  # noqa


class ShutdownProcess(ProcessTargetedEvent):
    """
    Event emitted when a process should begin shutting down.

    This event is handled by the launch.actions.ExecuteProcess action, see it
    for details on what happens when this is emitted.

    Also see ProcessTargetedEvent for details on how to target a specific
    process.
    """

    name = 'launch.events.process.ShutdownProcess'

    def __init__(self, *, process_matcher: Callable[['ExecuteProcess'], bool]) -> None:
        """Constructor."""
        super().__init__(process_matcher=process_matcher)
