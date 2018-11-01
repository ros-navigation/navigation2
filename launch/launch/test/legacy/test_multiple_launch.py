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
import sys

from launch.legacy import LaunchDescriptor
from launch.legacy.launcher import DefaultLauncher


def test_multiple_launch():
    launch(1)
    launch(2)


def launch(index):
    default_launcher = DefaultLauncher()

    async def coroutine():
        await asyncio.sleep(1)
        print('message %d' % index, file=sys.stderr)

    launch_descriptor = LaunchDescriptor()
    launch_descriptor.add_coroutine(coroutine(), name='coroutine%d' % index)

    print('launch %d' % index, file=sys.stderr)
    default_launcher.add_launch_descriptor(launch_descriptor)
    rc = default_launcher.launch()
    print('done %d' % index, rc, file=sys.stderr)


if __name__ == '__main__':
    test_multiple_launch()
