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

import os
import sys
import tempfile

from launch.legacy import LaunchDescriptor
from launch.legacy.exit_handler import ignore_exit_handler
from launch.legacy.launcher import DefaultLauncher
from launch_testing import create_handler


def test_matching():
    output_handlers = []

    launch_descriptor = LaunchDescriptor()

    # This temporary directory and files contained in it will be deleted when the process ends.
    tempdir = tempfile.mkdtemp()
    output_file = tempdir + os.sep + 'testfile'
    full_output_file = output_file + '.regex'
    with open(full_output_file, 'w+') as f:
        f.write('this is line \d\nthis is line [a-z]')

    name = 'test_executable_0'

    handler = create_handler(name, launch_descriptor, output_file)

    assert handler, 'Cannot find appropriate handler for %s' % output_file

    output_handlers.append(handler)

    executable_command = [
        sys.executable,
        os.path.join(os.path.abspath(os.path.dirname(__file__)), 'matching.py')]

    launch_descriptor.add_process(
        cmd=executable_command,
        name=name,
        exit_handler=ignore_exit_handler,
        output_handlers=output_handlers)

    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()

    assert rc == 0, \
        "The launch file failed with exit code '" + str(rc) + "'. "

    for handler in output_handlers:
        handler.check()


if __name__ == '__main__':
    test_matching()
