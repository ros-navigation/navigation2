#! /usr/bin/env python3
# Copyright 2024 Open Navigation LLC
# Copyright 2024 Stevedan Ogochukwu Omodolor Omodia
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

"""General utility functions."""

import os
import signal
import subprocess


def find_os_processes(name):
    """Find all the processes that are running gz sim."""
    ps_output = subprocess.check_output(['ps', 'aux'], text=True)
    ps_lines = ps_output.split('\n')
    gz_sim_processes = []
    for line in ps_lines:
        if name in line:
            columns = line.split()
            pid = columns[1]
            command = ' '.join(columns[10:])
            if command.startswith(name):
                gz_sim_processes.append((pid, command))
    return gz_sim_processes


def kill_process(pid):
    """Kill a process with a given PID."""
    try:
        os.kill(int(pid), signal.SIGKILL)
        print(f'Successfully killed process with PID: {pid}')
    except Exception as e:
        print(f'Failed to kill process with PID: {pid}. Error: {e}')


def kill_os_processes(name):
    """Kill all processes that are running with name."""
    processes = find_os_processes(name)
    if processes:
        for pid, _ in processes:
            kill_process(pid)
    else:
        print(f'No processes found starting with {name}')
