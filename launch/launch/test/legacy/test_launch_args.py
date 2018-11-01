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
from launch.legacy.arguments import get_launch_args


def test_launch_args():
    # Test empty arguments
    argv = []
    args = get_launch_args(argv)
    assert len(args) == 0, 'Failed empty list'

    # Test single argument with no launch arg
    argv = ['/path/to/my/executable']
    args = get_launch_args(argv)
    assert len(args) == 0, 'Failed single non-launch arg'

    # Test single argument with launch arg
    argv = ['my_arg:=12345']
    args = get_launch_args(argv)
    assert len(args) == 1, 'Failed single launch arg'
    assert 'my_arg' in args, 'Did not find my_arg'
    assert '12345' == args['my_arg'], 'my_arg has invalid value'

    # Test multiple non-launch args
    argv = ['/path/to/file', 'hello', '123', 'abcdefg']
    args = get_launch_args(argv)
    assert len(args) == 0, 'Failed multiple non-launch args'

    # Test multiple launch arguments
    argv = ['arg1:=12345', 'arg2:=true', 'arg3:=whatever', 'arg4:=yes']
    args = get_launch_args(argv)
    assert len(args) == 4, 'Failed multiple launch args'
    assert 'arg1' in args, 'Did not find arg1'
    assert 'arg2' in args, 'Did not find arg2'
    assert 'arg3' in args, 'Did not find arg3'
    assert 'arg4' in args, 'Did not find arg4'
    assert '12345' == args['arg1'], 'arg1 has invalid value'
    assert 'true' == args['arg2'], 'arg2 has invalid value'
    assert 'whatever' == args['arg3'], 'arg3 has invalid value'
    assert 'yes' == args['arg4'], 'arg4 has invalid value'

    # Test list of both non-launch args and args
    argv = [
        '/path/to/file',
        'arg5:=HELLO',
        '--help',
        'my_fancy_arg:=/cmd_vel',
        '-v',
        'verbose:=false',
    ]
    args = get_launch_args(argv)
    assert len(args) == 3, 'Failed list of both non-launch args and args'
    assert 'arg5' in args, 'Did not find arg5'
    assert 'my_fancy_arg' in args, 'Did not find my_fancy_arg'
    assert 'verbose' in args, 'Did not find verbose'
    assert 'HELLO' == args['arg5'], 'arg5 has invalid value'
    assert '/cmd_vel' == args['my_fancy_arg'], 'my_fancy_arg has invalid value'
    assert 'false' == args['verbose'], 'verbose has invalid value'

    # Test custom separator
    argv = [
        '/path/to/file',
        'arg5->HELLO',
        '--help',
        'my_fancy_arg->/cmd_vel',
        '-v', 'verbose:=false',
    ]
    args = get_launch_args(argv, separator='->')
    assert len(args) == 2, 'Failed custom separator'
    assert 'arg5' in args, 'Did not find arg5'
    assert 'my_fancy_arg' in args, 'Did not find my_fancy_arg'
    assert 'verbose' not in args, 'Found verbose but should not have'
    assert 'HELLO' == args['arg5'], 'arg5 has invalid value'
    assert '/cmd_vel' == args['my_fancy_arg'], 'my_fancy_arg has invalid value'

    # Test pair without key
    argv = [':=my_value']
    args = get_launch_args(argv)
    assert len(args) == 0, 'Failed pair without key'

    # Test pair without value
    argv = ['my_key:=']
    args = get_launch_args(argv)
    assert len(args) == 1, 'Failed pair without value'
    assert 'my_key' in args, 'Did not find my_key argument'
    assert '' == args['my_key'], 'my_key arg has incorrect value'

    # Test pair without key and value
    argv = [':=']
    args = get_launch_args(argv)
    assert len(args) == 0, 'Failed pair without key and value'

    # Test value containing separator
    argv = ['key:=value:=another_value']
    args = get_launch_args(argv)
    assert len(args) == 1, 'Failed value containing separator'
    assert 'key' in args, 'Did not find key'
    assert 'value:=another_value' == args['key'], 'key had incorrect value'


if __name__ == '__main__':
    test_launch_args()
