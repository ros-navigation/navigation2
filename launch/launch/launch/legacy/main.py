# Copyright 2015-2017 Open Source Robotics Foundation, Inc.
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

import argparse
import os
import sys

from launch.legacy import LaunchDescriptor
from launch.legacy.launcher import DefaultLauncher
from launch.legacy.loader import load_launch_file


def file_exists(filename):
    if not os.path.isfile(filename):
        raise argparse.ArgumentError("'%s' does not exist" % filename)
    return filename


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Launch the processes specified in a launch file.')
    parser.add_argument(
        'launch_file',
        type=file_exists,
        nargs='+',
        help='The launch file.')
    parser.add_argument(
        '--args',
        metavar='arg',
        type=str,
        nargs='+',
        help='An argument to the launch file (e.g., arg_name:=value). All '
             'arguments will be passed to each launch file.')
    args = parser.parse_args(argv)

    launcher = DefaultLauncher()
    for launch_file in args.launch_file:
        launch_descriptor = LaunchDescriptor()
        load_launch_file(launch_file, launch_descriptor, args.args)
        launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
    sys.exit(main())
