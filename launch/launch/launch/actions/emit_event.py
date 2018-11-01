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

"""Module for the EmitEvent action."""

from ..action import Action
from ..event import Event
from ..launch_context import LaunchContext
from ..utilities import is_a_subclass


class EmitEvent(Action):
    """Action that emits an event when executed."""

    def __init__(self, *, event: Event, **kwargs) -> None:
        """Constructor."""
        super().__init__(**kwargs)
        if not is_a_subclass(event, Event):
            raise RuntimeError("EmitEvent() expected an event instance, got '{}'.".format(event))
        self.__event = event

    @property
    def event(self):
        """Getter for self.__event."""
        return self.__event

    def execute(self, context: LaunchContext):
        """Execute the action."""
        context.emit_event_sync(self.__event)
