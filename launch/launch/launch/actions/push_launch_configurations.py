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

"""Module for the PushLaunchConfigurations action."""

from ..action import Action
from ..launch_context import LaunchContext


class PushLaunchConfigurations(Action):
    """
    Action that pushes the current state of launch configurations to a stack.

    The state can be restored by popping the stack with the
    :py:class:`launch.actions.PopLaunchConfigurations` action.
    """

    def __init__(self, **kwargs) -> None:
        """Constructor."""
        super().__init__(**kwargs)

    def execute(self, context: LaunchContext):
        """Execute the action."""
        context._push_launch_configurations()
