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

"""Python package utility functions related to loading Python Launch Files."""

from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec
from importlib.util import spec_from_loader
from types import ModuleType
from typing import Text

from ..launch_description import LaunchDescription


class InvalidPythonLaunchFileError(Exception):
    """Exception raised when the given Python launch file is not valid."""

    ...


def load_python_launch_file_as_module(python_launch_file_path: Text) -> ModuleType:
    """Load a given Python launch file (by path) as a Python module."""
    loader = SourceFileLoader('python_launch_file', python_launch_file_path)
    spec = spec_from_loader(loader.name, loader)
    mod = module_from_spec(spec)
    loader.exec_module(mod)
    return mod


def get_launch_description_from_python_launch_file(
    python_launch_file_path: Text
) -> LaunchDescription:
    """
    Load a given Python launch file (by path), and return the launch description from it.

    Python launch files are expected to have a `.py` extension and must provide
    a function within the module called `generate_launch_description()`.
    This function is called after loading the module to get the single
    :class:`launch.LaunchDescription` class from it.
    The signature of the function should be as follows:

    .. code-block:: python

        def generate_launch_description() -> launch.LaunchDescription:
            ...

    The Python launch file, as much as possible, should avoid side-effects.
    Keep in mind that the reason it is being loaded may be just to introspect
    the launch description and not necessarily to execute the launch itself.
    """
    launch_file_module = load_python_launch_file_as_module(python_launch_file_path)
    if not hasattr(launch_file_module, 'generate_launch_description'):
        raise InvalidPythonLaunchFileError(
            "launch file at '{}' does not contain the required function '{}'".format(
                python_launch_file_path, 'generate_launch_description()'
            ))
    return getattr(launch_file_module, 'generate_launch_description')()
