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


def get_launch_args(argv, separator=':='):
    """
    Get the list of launch arguments passed on the command line.

    Return a dictionary of key value pairs for launch arguments passed
    on the command line using the format 'key{separator}value'. This will
    process every string in the argv list.

    NOTE: all key value pairs will be returned as strings and no type
    checking is applied to the values.

    For example:

        argv = ['arg1:=123', 'hello', 'arg2:=true', 'arg3:=true', '/path/to/file']
        args = get_launch_args(argv)
        # Results in args = {'arg1': '123', 'arg2': 'true', 'arg3': 'true'}

    :param argv: is the list of string command line arguments
    :type argv: list(str)
    :param str separator: is the string separator for each key value pair (e.g., ':=')
    :returns: a dictionary of string key value pairs
    :rtype: dict(str, str)

    """
    launch_args = {}

    # Separate the command line arguments into a dictionary of
    # launch key value pairs of the format key${separator}value. Process the
    # first element even though it will likely be the executable name
    # to support other lists.
    for arg in argv:
        # Split the arg into a key value pair (allow values to contain
        # the separator)
        arg_pair = arg.split(separator, 1)
        if len(arg_pair) != 2:
            continue  # Skip non launch args

        key, value = arg_pair

        # Ignore pairs with an empty key, which isn't a useful argument pair
        if len(key) == 0:
            continue

        launch_args[key] = value

    return launch_args
