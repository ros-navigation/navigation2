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

"""Tests for the LaunchContext class."""

import asyncio

from launch import LaunchContext

import pytest


def test_launch_context_constructors():
    """Test the constructors for LaunchContext class."""
    LaunchContext()
    LaunchContext(argv=[])
    LaunchContext(argv=['--arg1', 'value'])


def test_launch_context_get_argv():
    """Test the getting of argv in the LaunchContext class."""
    argv = ['a', 'b']
    lc = LaunchContext(argv=argv)
    assert lc.argv == argv

    lc = LaunchContext()
    assert lc.argv == []


def test_launch_context_get_set_asyncio_loop():
    """Test the getting and settings for asyncio_loop in the LaunchContext class."""
    lc = LaunchContext()
    assert lc.asyncio_loop is None
    lc._set_asyncio_loop(asyncio.get_event_loop())
    assert lc.asyncio_loop == asyncio.get_event_loop()


def test_launch_context_locals():
    """Test "locals" feature of LaunchContext class."""
    lc = LaunchContext()

    assert len(lc.get_locals_as_dict()) == 0

    lc.extend_locals({'foo': 1})

    assert 'foo' in lc.get_locals_as_dict()
    assert lc.get_locals_as_dict()['foo'] == 1
    assert lc.locals.foo == 1
    with pytest.raises(AttributeError):
        lc.locals.foo = 2

    lc._push_locals()
    assert lc.locals.foo == 1
    lc.extend_locals({'foo': 2, 'bar': 3})
    assert lc.locals.foo == 2
    assert lc.locals.bar == 3
    lc._pop_locals()
    assert lc.locals.foo == 1
    with pytest.raises(AttributeError):
        assert lc.locals.bar == 3

    with pytest.raises(RuntimeError):
        lc._pop_locals()


def test_launch_context_launch_configurations():
    """Test "launch configurations" feature of LaunchContext class."""
    lc = LaunchContext()

    assert len(lc.launch_configurations) == 0

    lc.launch_configurations.update({'foo': 1})

    assert lc.launch_configurations['foo'] == 1

    lc._push_launch_configurations()
    assert lc.launch_configurations['foo'] == 1
    lc.launch_configurations.update({'foo': 2, 'bar': 3})
    assert lc.launch_configurations['foo'] == 2
    assert lc.launch_configurations['bar'] == 3
    lc._pop_launch_configurations()
    assert lc.launch_configurations['foo'] == 1
    assert 'bar' not in lc.launch_configurations

    with pytest.raises(RuntimeError):
        lc._pop_launch_configurations()


def test_launch_context_register_event_handlers():
    """Test registering of event handlers in LaunchContext class."""
    lc = LaunchContext()

    class MockEventHandler:
        ...

    mock_event_handler = MockEventHandler()
    lc.register_event_handler(mock_event_handler)
    assert len(lc._event_handlers) == 1
    assert lc._event_handlers[0] == mock_event_handler
    lc.unregister_event_handler(mock_event_handler)
    assert len(lc._event_handlers) == 0
    with pytest.raises(ValueError):
        lc.unregister_event_handler(mock_event_handler)


def test_launch_context_emit_events():
    """Test emitting events in LaunchContext class."""
    lc = LaunchContext()

    class MockEvent:
        name = 'MockEvent'

    assert lc._event_queue.qsize() == 0
    mock_event = MockEvent()
    lc.emit_event_sync(mock_event)
    assert lc._event_queue.qsize() == 1

    mock_event2 = MockEvent()
    loop = asyncio.get_event_loop()
    task = loop.create_task(lc.emit_event(mock_event2))
    loop.run_until_complete(task)
    assert lc._event_queue.qsize() == 2


def test_launch_context_perform_substitution():
    """Test performing substitutions with LaunchContext class."""
    lc = LaunchContext()

    from launch.substitutions import TextSubstitution

    sub = TextSubstitution(text='foo')
    assert lc.perform_substitution(sub) == 'foo'
