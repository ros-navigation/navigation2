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

from launch.legacy.exit_handler import default_exit_handler
from launch.legacy.output_handler import CompositeOutputHandler
from launch.legacy.output_handler import ConsoleOutput


class LaunchDescriptor:

    def __init__(self):
        self.task_descriptors = []

    def add_coroutine(self, coroutine, name=None, exit_handler=None):
        if name is not None and name in [p.name for p in self.task_descriptors]:
            raise RuntimeError("Task name '%s' already used" % name)
        if exit_handler is None:
            exit_handler = default_exit_handler
        coroutine_descriptor = CoroutineDescriptor(
            coroutine, name, exit_handler)
        self.task_descriptors.append(coroutine_descriptor)
        return coroutine_descriptor

    def add_process(self, cmd, name=None, env=None, output_handlers=None, exit_handler=None):
        if name is not None and name in [p.name for p in self.task_descriptors]:
            raise RuntimeError("Task name '%s' already used" % name)
        if output_handlers is None:
            output_handlers = [ConsoleOutput()]
        output_handlers = CompositeOutputHandler(output_handlers)
        if exit_handler is None:
            exit_handler = default_exit_handler
        process_descriptor = ProcessDescriptor(
            cmd, name, output_handlers, exit_handler, env=env)
        self.task_descriptors.append(process_descriptor)
        return process_descriptor


class TaskDescriptor:

    def __init__(self):
        self.task_state = None


class CoroutineDescriptor(TaskDescriptor):

    def __init__(self, coroutine, name, exit_handler):
        super(CoroutineDescriptor, self).__init__()
        self.coroutine = coroutine
        self.name = name
        self.exit_handler = exit_handler


class ProcessDescriptor(TaskDescriptor):

    def __init__(self, cmd, name, output_handler, exit_handler, env=None):
        super(ProcessDescriptor, self).__init__()
        self.cmd = cmd
        self.name = name
        self.output_handler = output_handler
        self.exit_handler = exit_handler
        self.env = env
        self.transport = None
        self.protocol = None

    def send_signal(self, signal):
        if self.transport:
            self.transport.send_signal(signal)

    def terminate(self):
        if self.transport:
            self.transport.terminate()
