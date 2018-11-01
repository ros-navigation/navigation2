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

"""Tests for the PopLaunchConfigurations and PushLaunchConfigurations action classes."""

from launch import LaunchContext
from launch.actions import PopLaunchConfigurations
from launch.actions import PushLaunchConfigurations


def test_push_and_pop_launch_configuration_constructors():
    """Test the constructors for PopLaunchConfigurations and PushLaunchConfigurations classes."""
    PopLaunchConfigurations()
    PushLaunchConfigurations()


def test_push_and_pop_launch_configuration_execute():
    """Test the execute() of the PopLaunchConfigurations and PushLaunchConfigurations classes."""
    lc1 = LaunchContext()

    # does not change empty state
    assert len(lc1.launch_configurations) == 0
    PushLaunchConfigurations().visit(lc1)
    PopLaunchConfigurations().visit(lc1)
    assert len(lc1.launch_configurations) == 0

    # does not change single config
    lc1.launch_configurations['foo'] = 'FOO'
    assert len(lc1.launch_configurations) == 1
    assert 'foo' in lc1.launch_configurations
    assert lc1.launch_configurations['foo'] == 'FOO'
    PushLaunchConfigurations().visit(lc1)
    PopLaunchConfigurations().visit(lc1)
    assert len(lc1.launch_configurations) == 1
    assert 'foo' in lc1.launch_configurations
    assert lc1.launch_configurations['foo'] == 'FOO'

    # does scope additions
    lc2 = LaunchContext()
    PushLaunchConfigurations().visit(lc2)
    lc2.launch_configurations['foo'] = 'FOO'
    PopLaunchConfigurations().visit(lc2)
    assert len(lc2.launch_configurations) == 0
    assert 'foo' not in lc2.launch_configurations

    # does scope modifications
    lc3 = LaunchContext()
    lc3.launch_configurations['foo'] = 'FOO'
    PushLaunchConfigurations().visit(lc3)
    lc3.launch_configurations['foo'] = 'BAR'
    PopLaunchConfigurations().visit(lc3)
    assert len(lc3.launch_configurations) == 1
    assert 'foo' in lc3.launch_configurations
    assert lc3.launch_configurations['foo'] == 'FOO'

    # does scope deletions
    lc4 = LaunchContext()
    lc4.launch_configurations['foo'] = 'FOO'
    PushLaunchConfigurations().visit(lc4)
    del lc4.launch_configurations['foo']
    PopLaunchConfigurations().visit(lc4)
    assert len(lc4.launch_configurations) == 1
    assert 'foo' in lc4.launch_configurations
    assert lc4.launch_configurations['foo'] == 'FOO'
