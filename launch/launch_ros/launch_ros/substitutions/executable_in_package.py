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

"""Module for the ExecutableInPackage substitution."""

import os
from typing import List
from typing import Text

from ament_index_python.packages import get_package_prefix

from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions.substitution_failure import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

from osrf_pycommon.process_utils import which


class ExecutableInPackage(Substitution):
    """
    Substitution that tries to locate an executable in the libexec directory of a ROS package.

    The ROS package is located using ament_index_python.

    :raise: ament_index_python.packages.PackageNotFoundError when package is
        not found during substitution
    :raise: SubstitutionFailure when executable is not found, or package
        libexec directory doesn't exist, during substitution
    """

    def __init__(self, executable: SomeSubstitutionsType, package: SomeSubstitutionsType) -> None:
        """Constructor."""
        super().__init__()
        self.__executable = normalize_to_list_of_substitutions(executable)
        self.__package = normalize_to_list_of_substitutions(package)

    @property
    def executable(self) -> List[Substitution]:
        """Getter for executable."""
        return self.__executable

    @property
    def package(self) -> List[Substitution]:
        """Getter for package."""
        return self.__package

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        exec_str = ' + '.join([sub.describe() for sub in self.executable])
        pkg_str = ' + '.join([sub.describe() for sub in self.package])
        return 'ExecInPkg(pkg={}, exec={})'.format(pkg_str, exec_str)

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by locating the executable."""
        executable = perform_substitutions(context, self.executable)
        package = perform_substitutions(context, self.package)
        package_prefix = get_package_prefix(package)
        package_libexec = os.path.join(package_prefix, 'lib', package)
        if not os.path.exists(package_libexec):
            raise SubstitutionFailure(
                "package '{}' found at '{}', but libexec directory '{}' does not exist".format(
                    package, package_prefix, package_libexec))
        result = which(executable, path=package_libexec)
        if result is None:
            raise SubstitutionFailure(
                "executable '{}' not found on the libexec directory '{}' ".format(
                    executable, package_libexec))
        return result
