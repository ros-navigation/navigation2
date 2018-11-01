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


class SubprocessProtocol(asyncio.SubprocessProtocol):

    def __init__(self, output_handler, *args, **kwargs):
        self.output_handler = output_handler
        self.exit_future = asyncio.Future()

        self.stdin = None
        self.stdout = None
        self.stderr = None

        asyncio.SubprocessProtocol.__init__(self, *args, **kwargs)

    def connection_made(self, transport):
        self.stdin = transport.get_pipe_transport(0)
        self.stdout = transport.get_pipe_transport(1)
        self.stderr = transport.get_pipe_transport(2)

    def pipe_data_received(self, fd, data):
        # This function is only called when pty's are not being used
        stdout = self.stdout
        if not isinstance(stdout, int):
            stdout = 1
        if fd == stdout:
            if hasattr(self, 'on_stdout_received'):
                self.on_stdout_received(data)
        else:
            assert fd == 2
            if hasattr(self, 'on_stderr_received'):
                self.on_stderr_received(data)

    def on_stdout_received(self, data):
        self.output_handler.on_stdout_received(data)

    def on_stderr_received(self, data):
        self.output_handler.on_stderr_received(data)

    def process_exited(self):
        self.output_handler.flush()
        self.exit_future.set_result(0)
