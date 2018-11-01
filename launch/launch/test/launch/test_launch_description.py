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

"""Tests for the LaunchDescription class."""

import collections
import logging
import os

from launch import Action
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch import LaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

logging.getLogger('launch').setLevel(logging.DEBUG)


def test_launch_description_constructors():
    """Test the constructors for LaunchDescription class."""
    LaunchDescription()
    LaunchDescription(None)
    LaunchDescription([])
    ld = LaunchDescription([LaunchDescriptionEntity()])
    assert len(ld.entities) == 1


def test_launch_description_get_launch_arguments():
    """Test the get_launch_arguments() method of the LaunchDescription class."""
    ld = LaunchDescription([])
    assert len(ld.get_launch_arguments()) == 0

    ld = LaunchDescription([DeclareLaunchArgument('foo')])
    la = ld.get_launch_arguments()
    assert len(la) == 1
    assert la[0]._conditionally_included is False

    ld = LaunchDescription([DeclareLaunchArgument('foo', condition=IfCondition('True'))])
    la = ld.get_launch_arguments()
    assert len(la) == 1
    assert la[0]._conditionally_included is True

    ld = LaunchDescription([
        IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription([
            DeclareLaunchArgument('foo'),
        ]))),
    ])
    la = ld.get_launch_arguments()
    assert len(la) == 1
    assert la[0]._conditionally_included is False

    this_dir = os.path.dirname(os.path.abspath(__file__))
    ld = LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(this_dir, 'launch_file_with_argument.launch.py'))),
    ])
    la = ld.get_launch_arguments()
    assert len(la) == 1
    assert la[0]._conditionally_included is False

    ld = LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([
            # This will prevent loading of this launch file to find arguments in it.
            ThisLaunchFileDir(),
            'launch_file_with_argument.launch.py',
        ])),
    ])
    la = ld.get_launch_arguments()
    assert len(la) == 0


def test_launch_description_add_things():
    """Test adding things to the LaunchDescription class."""
    ld = LaunchDescription()
    assert len(ld.entities) == 0
    ld.add_entity(LaunchDescription())
    assert len(ld.entities) == 1
    ld.add_action(Action())
    assert len(ld.entities) == 2


def test_launch_description_visit():
    """Test visiting entities in the LaunchDescription class."""
    ld = LaunchDescription([LaunchDescriptionEntity()])
    ld.add_action(Action())

    class MockLaunchContext:
        ...

    result = ld.visit(MockLaunchContext())
    assert isinstance(result, collections.Iterable)
    for entity in result:
        assert isinstance(entity, LaunchDescriptionEntity)
