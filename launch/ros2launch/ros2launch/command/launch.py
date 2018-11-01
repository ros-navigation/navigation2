# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from argparse import REMAINDER
import os
import sys

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError
from ros2cli.command import CommandExtension
from ros2launch.api import get_share_file_path_from_package
from ros2launch.api import InvalidPythonLaunchFileError
from ros2launch.api import launch_a_python_launch_file
from ros2launch.api import LaunchFileNameCompleter
from ros2launch.api import MultipleLaunchFilesError
from ros2launch.api import print_a_python_launch_file
from ros2launch.api import print_arguments_of_python_launch_file
from ros2pkg.api import package_name_completer


class LaunchCommand(CommandExtension):
    """Run a launch file."""

    def add_arguments(self, parser, cli_name):
        """Add arguments to argparse."""
        parser.add_argument(
            '-d', '--debug', default=False, action='store_true',
            help='Put the launch system in debug mode, provides more verbose output.')
        command_group = parser.add_mutually_exclusive_group()
        command_group.add_argument(
            '-p', '--print', '--print-description', default=False, action='store_true',
            help='Print the launch description to the console without launching it.')
        command_group.add_argument(
            '-s', '--show-args', '--show-arguments', default=False, action='store_true',
            help='Show arguments that may be given to the launch file.')
        arg = parser.add_argument(
            'package_name',
            help='Name of the ROS package which contains the launch file')
        arg.completer = package_name_completer
        arg = parser.add_argument(
            'launch_file_name',
            # TODO(wjwwood) make this not optional when full launch path is supported.
            nargs='?',
            help='Name of the launch file')
        arg = parser.add_argument(
            'launch_arguments',
            nargs='*',
            help="Arguments to the launch file; '<name>:=<value>' (for duplicates, last one wins)")
        arg.completer = LaunchFileNameCompleter()
        parser.add_argument(
            'argv', nargs=REMAINDER,
            help='Pass arbitrary arguments to the launch file')

    def main(self, *, parser, args):
        """Entry point for CLI program."""
        mode = 'pkg file'
        if args.launch_file_name is None:
            # If only one argument passed, use single file mode.
            mode = 'single file'
        else:
            # Test if first argument is a package, and if not change to single
            # file mode, but only if the file exists.
            try:
                get_package_prefix(args.package_name)
            except PackageNotFoundError:
                if os.path.exists(args.package_name):
                    mode = 'single file'

        path = None
        launch_arguments = []
        if mode == 'single file':
            # TODO(wjwwood): figure out how to have argparse and argcomplete
            # handle this, for now, hidden feature.
            if os.path.exists(args.package_name):
                path = args.package_name
            else:
                return 'No launch file supplied'

            if args.launch_file_name is not None:
                # Since in single file mode, the "launch file" argument is
                # actually part of the launch arguments, if set.
                launch_arguments.append(args.launch_file_name)
        elif mode == 'pkg file':
            try:
                path = get_share_file_path_from_package(
                    package_name=args.package_name,
                    file_name=args.launch_file_name)
            except PackageNotFoundError as exc:
                raise RuntimeError(
                    "Package '{}' not found: {}".format(args.package_name, exc))
            except (FileNotFoundError, MultipleLaunchFilesError) as exc:
                raise RuntimeError(str(exc))
        else:
            raise RuntimeError('unexpected mode')
        launch_arguments.extend(args.launch_arguments)
        try:
            if args.print:
                return print_a_python_launch_file(python_launch_file_path=path)
            elif args.show_args:
                return print_arguments_of_python_launch_file(python_launch_file_path=path)
            else:
                return launch_a_python_launch_file(
                    python_launch_file_path=path,
                    launch_file_arguments=launch_arguments,
                    debug=args.debug
                )
        except SyntaxError:
            print("""
Notice: SyntaxError (or related errors) can occur if your launch file ('{}') is not a Python file.
""".format(path), file=sys.stderr)
            raise  # raise so the user can see the traceback in useful SyntaxError's
        except ValueError as exc:
            print("""
Notice: ValueError can occur if your launch file ('{}') is a binary file and not a Python file.
""".format(path), file=sys.stderr)
            raise RuntimeError('ValueError: {}'.format(str(exc)))
        except InvalidPythonLaunchFileError as exc:
            # TODO(wjwwood): refactor this after we deprecate and then remove the old launch
            print("""
Notice: Your launch file may have been designed to be used with an older version of ROS 2.
Or that the file you specified is Python code, but not a launch file.
""", file=sys.stderr)
            raise RuntimeError('InvalidPythonLaunchFileError: {}'.format(str(exc)))
