# Copyright 2015 Open Source Robotics Foundation, Inc.
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

import asyncio
import os
import sys

from launch.legacy import LaunchDescriptor
from launch.legacy.exit_handler import primary_exit_handler
from launch.legacy.launcher import DefaultLauncher
from launch.legacy.loader import load_launch_file


def test_launch_with_coroutine():
    default_launcher = DefaultLauncher()

    launch_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'launch_counter.py')
    launch_descriptor = LaunchDescriptor()
    load_launch_file(launch_file, launch_descriptor, {})

    async def coroutine():
        await asyncio.sleep(1)
        print('one', file=sys.stderr)
        await asyncio.sleep(1)
        print('two', file=sys.stderr)
        await asyncio.sleep(1)
        print('three', file=sys.stderr)

    async def coroutine2():
        await asyncio.sleep(1)
        print('one mississippi', file=sys.stderr)
        await asyncio.sleep(1)
        print('two mississippi', file=sys.stderr)
        await asyncio.sleep(1)
        print('three mississippi', file=sys.stderr)

    launch_descriptor.add_coroutine(
        coroutine(), name='coroutine', exit_handler=primary_exit_handler)
    # launch_descriptor.add_coroutine(coroutine2())

    print('launch', file=sys.stderr)
    default_launcher.add_launch_descriptor(launch_descriptor)
    rc = default_launcher.launch()
    print('done', rc, file=sys.stderr)


if __name__ == '__main__':
    test_launch_with_coroutine()
