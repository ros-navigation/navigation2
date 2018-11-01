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

"""Tests for the LaunchDescriptionEntity class."""

from launch import LaunchDescriptionEntity

import pytest


def test_launch_description_entity_constructors():
    """Test the constructors for LaunchDescriptionEntity class."""
    LaunchDescriptionEntity()


def test_launch_description_entity_methods():
    """Test the methods of the LaunchDescriptionEntity class."""
    class MockLaunchContext:
        ...

    lde = LaunchDescriptionEntity()
    with pytest.raises(NotImplementedError):
        lde.describe()
    assert isinstance(lde.describe_sub_entities(), list)
    assert isinstance(lde.describe_conditional_sub_entities(), list)
    assert lde.visit(MockLaunchContext()) is None
    assert lde.get_asyncio_future() is None
