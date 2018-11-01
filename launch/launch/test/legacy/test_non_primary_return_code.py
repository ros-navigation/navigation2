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


def test_non_primary_return_code():
    # since Python < 3.5 on Windows does not support signaling SIGINT to the subprocesses
    # we can't expect them to shutdown cleanly, therefore we ignore this test
    if os.name == 'nt':
        return

    default_launcher = DefaultLauncher()

    async def coroutine1():
        await asyncio.sleep(1)
        print('one', file=sys.stderr)
        await asyncio.sleep(1)
        print('two', file=sys.stderr)
        return 3

    async def coroutine2():
        await asyncio.sleep(1)
        print('one mississippi', file=sys.stderr)
        return 0

    launch_descriptor = LaunchDescriptor()
    launch_descriptor.add_coroutine(coroutine1(), name='coroutine1')
    launch_descriptor.add_coroutine(
        coroutine2(), name='coroutine2', exit_handler=primary_exit_handler)

    print('launch', file=sys.stderr)
    default_launcher.add_launch_descriptor(launch_descriptor)
    rc = default_launcher.launch()
    print('done', rc, file=sys.stderr)
    assert rc == 3, 'Expected return code is 3'


if __name__ == '__main__':
    test_non_primary_return_code()
