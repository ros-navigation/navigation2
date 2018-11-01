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

"""Tests for the LaunchDescriptionSource class."""

from launch import LaunchDescription
from launch import LaunchDescriptionSource

import pytest


def test_launch_description_source_constructors():
    """Test the constructors for LaunchDescriptionSource class."""
    LaunchDescriptionSource()
    LaunchDescriptionSource(LaunchDescription())
    LaunchDescriptionSource(LaunchDescription(), 'location')
    LaunchDescriptionSource(LaunchDescription(), 'location', 'method description')


def test_launch_description_source_methods():
    """Test the methods of the LaunchDescriptionSource class."""
    class MockLaunchContext:
        ...

    lds = LaunchDescriptionSource()
    with pytest.raises(RuntimeError):
        lds.get_launch_description(MockLaunchContext())

    ld = LaunchDescription()
    lds = LaunchDescriptionSource(ld)
    assert lds.get_launch_description(MockLaunchContext()) == ld

    ld = LaunchDescription()
    lds = LaunchDescriptionSource(ld, 'location')
    assert lds.get_launch_description(MockLaunchContext()) == ld
    assert lds.location == 'location'

    ld = LaunchDescription()
    lds = LaunchDescriptionSource(ld, 'location', 'method description')
    assert lds.get_launch_description(MockLaunchContext()) == ld
    assert lds.location == 'location'
    assert lds.method == 'method description'
