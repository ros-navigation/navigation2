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

"""Module for the PythonLaunchDescriptionSource class."""

import logging
import traceback
from typing import Optional
from typing import Text  # noqa: F401

from .python_launch_file_utilities import get_launch_description_from_python_launch_file
from ..launch_context import LaunchContext
from ..launch_description import LaunchDescription
from ..some_substitutions_type import SomeSubstitutionsType
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions

_logger = logging.getLogger('launch.launch_description_sources.PythonLaunchDescriptionSource')


class PythonLaunchDescriptionSource:
    """Encapsulation of a Python launch file, which can be loaded during launch."""

    def __init__(
        self,
        launch_file_path: SomeSubstitutionsType,
    ) -> None:
        """
        Constructor.

        The given file path should be to a ``.launch.py`` style file.
        The path should probably be absolute, since the current working
        directory will be wherever the launch file was run from, which might
        change depending on the situation.
        The path can be made up of Substitution instances which are expanded
        when :py:meth:`get_launch_description()` is called.

        See also :py:func:`get_launch_description_from_python_launch_file`.

        :param launch_file_path: the path to the launch file
        """
        self.__launch_file_path = normalize_to_list_of_substitutions(launch_file_path)
        self.__expanded_launch_file_path = None  # type: Optional[Text]
        self.__launch_description = None  # type: Optional[LaunchDescription]

    def try_get_launch_description_without_context(self) -> Optional[LaunchDescription]:
        """Get the LaunchDescription, attempting to load it if necessary."""
        if self.__launch_description is None:
            # Try to expand the launch file path and load the launch file with a local context.
            try:
                context = LaunchContext()
                expanded_launch_file_path = \
                    perform_substitutions(context, self.__launch_file_path)
                return get_launch_description_from_python_launch_file(expanded_launch_file_path)
            except Exception as exc:
                _logger.debug(traceback.format_exc())
                _logger.debug('Failed to load the launch file without a context: ' + str(exc))
        return self.__launch_description

    def get_launch_description(self, context: LaunchContext) -> LaunchDescription:
        """Get the LaunchDescription, loading it if necessary."""
        if self.__expanded_launch_file_path is None:
            self.__expanded_launch_file_path = \
                perform_substitutions(context, self.__launch_file_path)
        if self.__launch_description is None:
            self.__launch_description = \
                get_launch_description_from_python_launch_file(self.__expanded_launch_file_path)
        return self.__launch_description

    @property
    def location(self) -> str:
        """
        Get the location of the Python launch file as a string.

        The string is either a list of Substitution instances converted to
        strings or the expanded path if :py:meth:`get_launch_description` has
        been called.
        """
        if self.__expanded_launch_file_path is None:
            # get_launch_description() has not been called yet
            return ' + '.join([str(sub) for sub in self.__launch_file_path])
        return self.__expanded_launch_file_path

    @property
    def method(self) -> str:
        """Getter for self.__method."""
        return 'interpreted python launch file'
