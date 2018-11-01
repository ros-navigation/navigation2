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

"""Module for the ThisLaunchFileDir substitution."""

from typing import Text

from .substitution_failure import SubstitutionFailure
from ..launch_context import LaunchContext
from ..substitution import Substitution


class ThisLaunchFileDir(Substitution):
    """Substitution that returns the absolute path to the current launch file."""

    def __init__(self) -> None:
        """Constructor."""
        super().__init__()

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'ThisLaunchFileDir()'

    def perform(self, context: LaunchContext) -> Text:
        """
        Perform the substitution by returning the path to the current launch file.

        If there is no current launch file, i.e. if run from a script, then an
        error is raised.

        :raises: SubstitutionFailure if not in a launch file
        """
        if 'current_launch_file_directory' not in context.get_locals_as_dict():
            raise SubstitutionFailure(
                'ThisLaunchFileDir used outside of a launch file (in a script)')
        return context.locals.current_launch_file_directory
