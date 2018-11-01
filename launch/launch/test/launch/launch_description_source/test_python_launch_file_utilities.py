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

"""Tests for the Python launch file utility functions."""

import os
import sys

from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch.launch_description_sources import InvalidPythonLaunchFileError
from launch.launch_description_sources import load_python_launch_file_as_module

import pytest


def test_load_python_launch_file_as_module():
    """Test load_python_launch_file_as_module()."""
    this_dir = os.path.dirname(os.path.abspath(__file__))
    module = load_python_launch_file_as_module(os.path.join(this_dir, 'loadable_python_module.py'))
    assert sys.executable == module.some_function()

    with pytest.raises(FileNotFoundError):
        module = load_python_launch_file_as_module('does_not_exist')


def test_get_launch_description_from_python_launch_file():
    """Test get_launch_description_from_python_launch_file()."""
    this_dir = os.path.dirname(os.path.abspath(__file__))
    ld = get_launch_description_from_python_launch_file(os.path.join(this_dir, 'simple.launch.py'))
    assert 0 == len(ld.entities)

    with pytest.raises(InvalidPythonLaunchFileError):
        ld = get_launch_description_from_python_launch_file(
            os.path.join(this_dir, 'loadable_python_module.py'))

    with pytest.raises(FileNotFoundError):
        ld = get_launch_description_from_python_launch_file('does_not_exist')
