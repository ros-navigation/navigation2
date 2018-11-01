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

"""Tests for the LaunchService class."""

import queue
import threading

from launch import LaunchDescription
from launch import LaunchService
from launch.utilities import install_signal_handlers

# Install the signal handlers here, in the hope that this is executed in the
# main-thread.
# If this is not the main-thread, a ValueError will be raised.
# See the docs for this function for more info.
# This would ensure that the custom handlers that the LaunchService needs can
# be installed later from other threads.
install_signal_handlers()


def test_launch_service_constructors():
    """Test the constructors for LaunchService class."""
    LaunchService()
    LaunchService(debug=True)
    LaunchService(debug=False)


def test_launch_service_emit_event():
    """
    Test the emitting of events in the LaunchService class.

    Also covers basic tests for include_launch_description(), run(), and
    shutdown().
    """
    ls = LaunchService(debug=True)

    assert ls._LaunchService__context._event_queue.qsize() == 0

    from launch.actions import OpaqueFunction
    from launch.actions import RegisterEventHandler
    from launch.event_handler import EventHandler

    handled_events = queue.Queue()
    ld = LaunchDescription([
        RegisterEventHandler(EventHandler(
            matcher=lambda event: True,
            entities=OpaqueFunction(
                function=lambda context: handled_events.put(context.locals.event),
            ),
        ))
    ])
    ls.include_launch_description(ld)
    assert ls._LaunchService__context._event_queue.qsize() == 1

    class MockEvent:
        name = 'Event'

    ls.emit_event(MockEvent())
    assert ls._LaunchService__context._event_queue.qsize() == 2
    assert handled_events.qsize() == 0

    t = threading.Thread(target=ls.run, kwargs={'shutdown_when_idle': False})
    t.start()

    # First event (after including description of event handler).
    handled_events.get(block=True, timeout=5.0)

    # Emit and then check for a second event.
    ls.emit_event(MockEvent())
    handled_events.get(block=True, timeout=5.0)

    # Shutdown (generates a third event) and join the thread.
    ls.shutdown()
    t.join()
    # Check that the shutdown event was handled.
    handled_events.get(block=False)

    assert handled_events.qsize() == 0
    ls.emit_event(MockEvent())
    assert handled_events.qsize() == 0

    assert ls.run(shutdown_when_idle=True) == 0
    handled_events.get(block=False)
