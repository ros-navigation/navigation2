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
from tempfile import NamedTemporaryFile

from launch.legacy.exit_handler import ignore_exit_handler
from launch.legacy.output_handler import FileOutput


def launch(launch_descriptor, argv):
    counter_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'counter.py')

    with NamedTemporaryFile(mode='w', prefix='foo_', delete=False) as h:
        foo_filename = h.name

    ld = launch_descriptor
    ld.add_process(
        cmd=[sys.executable, '-u', counter_file, '--limit', '15', '--sleep', '0.5'],
        name='foo',
        output_handlers=[FileOutput(filename=foo_filename)],
        exit_handler=ignore_exit_handler,
    )
