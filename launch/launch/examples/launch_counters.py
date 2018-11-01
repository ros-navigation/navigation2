#!/usr/bin/env python3

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

"""
Script that demonstrates launch and subprocesses of varying behavior.

For an example of expected output, see the file next to this one called
"launch_counter_good_bad_ugly.example.txt".
"""

import os
import platform
import sys
from typing import cast
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa

import launch
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService
import launch.actions
import launch.events
import launch.substitutions


def main(argv=sys.argv[1:]):
    """Main."""
    # Any number of actions can optionally be given to the constructor of LaunchDescription.
    # Or actions/entities can be added after creating the LaunchDescription.
    user_env_var = 'USERNAME' if platform.system() == 'Windows' else 'USER'
    ld = LaunchDescription([
        launch.actions.LogInfo(msg='Hello World!'),
        launch.actions.LogInfo(msg=(
            'Is that you, ', launch.substitutions.EnvironmentVariable(name=user_env_var), '?'
        )),
    ])

    # Setup a custom event handler for all stdout/stderr from processes.
    # Later, this will be a configurable, but always present, extension to the LaunchService.
    def on_output(event: launch.Event) -> None:
        for line in event.text.decode().splitlines():
            print('[{}] {}'.format(
                cast(launch.events.process.ProcessIO, event).process_name, line))

    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        # this is the action     ^              and this, the event handler ^
        on_stdout=on_output,
        on_stderr=on_output,
    )))

    # Run whoami, and use its output to log the name of the user.
    # Prefix just the whoami process with `time`.
    ld.add_action(launch.actions.SetLaunchConfiguration('launch-prefix', 'time'))
    # Run whoami, but keep handle to action to make a targeted event handler.
    if platform.system() == 'Windows':
        whoami_cmd = ['echo', '%USERNAME%']
    else:
        whoami_cmd = [launch.substitutions.FindExecutable(name='whoami')]
    whoami_action = launch.actions.ExecuteProcess(
        cmd=whoami_cmd,
        shell=(platform.system() == 'Windows')
    )
    ld.add_action(whoami_action)
    # Make event handler that uses the output.
    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        target_action=whoami_action,
        # The output of `time` will be skipped since `time`'s output always goes to stderr.
        on_stdout=lambda event: launch.actions.LogInfo(
            msg="whoami says you are '{}'.".format(event.text.decode().strip())
        ),
    )))
    # Unset launch prefix to prevent other process from getting this setting.
    ld.add_action(launch.actions.SetLaunchConfiguration('launch-prefix', ''))

    # Run the counting program, with default options.
    counter_action = launch.actions.ExecuteProcess(cmd=[sys.executable, '-u', './counter.py'])
    ld.add_action(counter_action)

    # Setup an event handler for just this process which will exit when `Counter: 4` is seen.
    def counter_output_handler(event):
        target_str = 'Counter: 4'
        if target_str in event.text.decode():
            return launch.actions.EmitEvent(event=launch.events.Shutdown(
                reason="saw '{}' from '{}'".format(target_str, event.process_name)
            ))

    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        target_action=counter_action,
        on_stdout=counter_output_handler,
        on_stderr=counter_output_handler,
    )))

    # Run the counter a few more times, with various options.
    ld.add_action(launch.actions.ExecuteProcess(
        cmd=[sys.executable, '-u', './counter.py', '--ignore-sigint']
    ))
    ld.add_action(launch.actions.ExecuteProcess(
        cmd=[sys.executable, '-u', './counter.py', '--ignore-sigint', '--ignore-sigterm']
    ))

    # Add our own message for when shutdown is requested.
    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnShutdown(
        on_shutdown=[launch.actions.LogInfo(msg=[
            'Launch was asked to shutdown: ',
            launch.substitutions.LocalSubstitution('event.reason'),
        ])],
    )))

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    # ls = LaunchService(argv=argv, debug=True)  # Use this instead to get more debug messages.
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    sys.exit(main())
