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

"""Tests for the SetLaunchConfiguration and UnsetLaunchConfiguration action classes."""

from launch import LaunchContext
from launch.actions import SetLaunchConfiguration
from launch.actions import UnsetLaunchConfiguration
from launch.substitutions import LaunchConfiguration


def test_set_and_unset_launch_configuration_constructors():
    """Test the constructors for SetLaunchConfiguration and UnsetLaunchConfiguration classes."""
    SetLaunchConfiguration('name', 'value')
    UnsetLaunchConfiguration('value')


def test_set_and_unset_launch_configuration_execute():
    """Test the execute() of the SetLaunchConfiguration and UnsetLaunchConfiguration classes."""
    lc1 = LaunchContext()

    assert len(lc1.launch_configurations) == 0
    SetLaunchConfiguration('name', 'value').visit(lc1)
    assert len(lc1.launch_configurations) == 1
    assert 'name' in lc1.launch_configurations
    UnsetLaunchConfiguration('name').visit(lc1)
    assert len(lc1.launch_configurations) == 0
    assert 'name' not in lc1.launch_configurations

    # use substitutions
    assert len(lc1.launch_configurations) == 0
    SetLaunchConfiguration('foo', 'FOO').visit(lc1)
    SetLaunchConfiguration(['name-', LaunchConfiguration('foo')], 'value').visit(lc1)
    assert len(lc1.launch_configurations) == 2
    assert any(c.startswith('name-') for c in lc1.launch_configurations)
    UnsetLaunchConfiguration(['name-', LaunchConfiguration('foo')]).visit(lc1)
    assert len(lc1.launch_configurations) == 1
    assert not any(c.startswith('name-') for c in lc1.launch_configurations)
    UnsetLaunchConfiguration('foo').visit(lc1)
    assert len(lc1.launch_configurations) == 0

    # unset a non-existent config does not fail
    assert len(lc1.launch_configurations) == 0
    UnsetLaunchConfiguration('does_not_exist').visit(lc1)
