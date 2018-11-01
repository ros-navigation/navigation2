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

"""Tests for the RegisterEventHandler and UnregisterEventHandler action classes."""

from launch import EventHandler
from launch import LaunchContext
from launch.actions import RegisterEventHandler
from launch.actions import UnregisterEventHandler

import pytest


def test_register_event_handler_constructor():
    """Test the constructor for the RegisterEventHandler action class."""
    event_handler = EventHandler(matcher=lambda: True)
    register_event_handler_action = RegisterEventHandler(event_handler)
    assert event_handler == register_event_handler_action.event_handler


def test_register_unregister_event_handler_execute():
    """
    Test the execute method.

    Tests both the RegisterEventHandler and UnregisterEventHandler action classes.
    """
    launch_context = LaunchContext()
    event_handler = EventHandler(matcher=lambda: True)

    # Register event handler
    register_event_handler_action = RegisterEventHandler(event_handler)
    register_event_handler_action.execute(launch_context)

    # Unregister the same event handler
    unregister_event_handler_action = UnregisterEventHandler(event_handler)
    unregister_event_handler_action.execute(launch_context)

    # Expect ValueError when trying to unregister the event handler a second time
    with pytest.raises(ValueError):
        unregister_event_handler_action.execute(launch_context)


def test_unregister_event_handler_constructor():
    """Test the constructor for the UnregisterEventHandler action class."""
    event_handler = EventHandler(matcher=lambda: True)
    unregister_event_handler_action = UnregisterEventHandler(event_handler)
    assert event_handler == unregister_event_handler_action.event_handler
