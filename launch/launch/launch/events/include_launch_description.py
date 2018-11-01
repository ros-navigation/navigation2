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

"""Module for IncludeLaunchDescription event."""

from ..event import Event
from ..launch_description import LaunchDescription


class IncludeLaunchDescription(Event):
    """Event that can be emitted to cause a LaunchDescription to be included."""

    name = 'launch.events.IncludeLaunchDescription'

    def __init__(self, launch_description: LaunchDescription) -> None:
        """Constructor."""
        self.__launch_description = launch_description

    @property
    def launch_description(self):
        """Getter for launch_description."""
        return self.__launch_description
