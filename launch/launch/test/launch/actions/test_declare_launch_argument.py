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

"""Tests for the DeclareLaunchArgument action class."""

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

import pytest


def test_declare_launch_argument_constructors():
    """Test the constructors for DeclareLaunchArgument class."""
    DeclareLaunchArgument('name')
    DeclareLaunchArgument('name', default_value='default value')
    DeclareLaunchArgument('name', default_value='default value', description='description')


def test_declare_launch_argument_methods():
    """Test the methods of the DeclareLaunchArgument class."""
    dla1 = DeclareLaunchArgument('name', default_value='default value', description='description')
    assert dla1.name == 'name'
    assert isinstance(dla1.default_value, list)
    assert dla1.description == 'description'
    assert 'DeclareLaunchArgument' in dla1.describe()
    assert isinstance(dla1.describe_sub_entities(), list)
    assert isinstance(dla1.describe_conditional_sub_entities(), list)

    dla2 = DeclareLaunchArgument('name')
    assert dla2.default_value is None
    assert dla2.description, 'description does not have a non-empty default value'


def test_declare_launch_argument_execute():
    """Test the execute (or visit) of the DeclareLaunchArgument class."""
    action1 = DeclareLaunchArgument('name')
    lc1 = LaunchContext()
    with pytest.raises(RuntimeError) as excinfo:
        action1.visit(lc1)
    assert 'Required launch argument' in str(excinfo.value)

    lc1.launch_configurations['name'] = 'value'
    assert action1.visit(lc1) is None

    action2 = DeclareLaunchArgument('name', default_value='value')
    lc2 = LaunchContext()
    assert action2.visit(lc2) is None
    assert lc1.launch_configurations['name'] == 'value'
