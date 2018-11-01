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

import sys


class OutputHandler:

    def __init__(self):
        self.print_mutex = None
        self.line_prefix = None

    def get_description(self):
        raise NotImplementedError()

    def set_print_mutex(self, print_mutex):
        self.print_mutex = print_mutex

    def set_line_prefix(self, line_prefix):
        self.line_prefix = line_prefix

    def process_init(self):
        pass

    def support_stderr2stdout(self):
        return False

    def on_message_received(self, lines):
        self.on_stdout_received(lines)

    def on_stdout_received(self, data):
        raise NotImplementedError()

    def on_stderr_received(self, data):
        raise NotImplementedError()

    def flush(self):
        pass

    def process_cleanup(self):
        pass


class CompositeOutputHandler:

    def __init__(self, output_handlers):
        self.output_handlers = output_handlers

    def get_description(self):
        return ', '.join([h.get_description() for h in self.output_handlers])

    def support_stderr2stdout(self):
        return all(h.support_stderr2stdout() for h in self.output_handlers)

    def __getattr__(self, name):
        def wrapper(*args, **kwargs):
            [getattr(h, name)(*args, **kwargs) for h in self.output_handlers]
        return wrapper


class LineOutput(OutputHandler):

    def __init__(self):
        super(LineOutput, self).__init__()
        self.left_over_stdout = b''
        self.left_over_stderr = b''

    def on_stdout_received(self, data):
        lines, self.left_over_stdout = self._process_incoming_lines(data, self.left_over_stdout)
        self.on_stdout_lines(lines)

    def on_stdout_lines(self, lines):
        raise NotImplementedError()

    def on_stderr_received(self, data):
        lines, self.left_over_stderr = self._process_incoming_lines(data, self.left_over_stderr)
        self.on_stderr_lines(lines)

    def on_stderr_lines(self, lines):
        raise NotImplementedError()

    def flush(self):
        if self.left_over_stdout:
            self.on_stdout_lines(self.left_over_stdout)
        if self.left_over_stderr:
            self.on_stderr_lines(self.left_over_stderr)

    def _process_incoming_lines(self, incoming, left_over):
        # This function takes the new data, the left over data from last time
        # and returns a list of complete lines (separated by sep) as well as
        # any sep trailing data for the next iteration
        # This function takes and returns bytes only (str in Python2)
        combined = (left_over + incoming)
        lines = combined.splitlines(True)
        if not lines:
            return None, left_over
        # Use splitlines because it is magic
        # comparing against os.linesep is not sufficient
        if lines[-1].splitlines() != [lines[-1]]:
            data = b''.join(lines)
            left_over = b''
        else:
            data = b''.join(lines[:-1])
            left_over = lines[-1]
        return data, left_over


class ConsoleOutput(LineOutput):

    def __init__(self, stderr_only=False, avoid_stderr2stdout=False):
        super(ConsoleOutput, self).__init__()
        self.stderr_only = stderr_only
        self.avoid_stderr2stdout = avoid_stderr2stdout

    def get_description(self):
        return '%s > console' % ('all' if not self.stderr_only else 'stderr')

    def support_stderr2stdout(self):
        return not self.stderr_only and not self.avoid_stderr2stdout

    def on_message_received(self, lines):
        pass

    def on_stdout_lines(self, lines):
        if self.stderr_only:
            return
        with self.print_mutex:
            for line in lines.splitlines():
                print(self.line_prefix + str(line.decode()))

    def on_stderr_lines(self, lines):
        with self.print_mutex:
            for line in lines.splitlines():
                print(self.line_prefix + str(line.decode()), file=sys.stderr)


class FileOutput(LineOutput):

    def __init__(self, filename=None, filename_stdout=None, filename_stderr=None):
        super(FileOutput, self).__init__()
        assert (
            (filename is not None) ^
            (filename_stdout is not None or filename_stderr is not None))
        if filename is not None:
            self.filename_stdout = filename
            self.filename_stderr = False  # implies redirection to stdout
        else:
            assert filename_stdout != filename_stderr
            self.filename_stdout = filename_stdout
            self.filename_stderr = filename_stderr

        self.handle_stdout = None
        self.handle_stderr = None

    def get_description(self):
        if self.filename_stderr is False:
            return 'all > file:' + self.filename_stdout
        descriptions = []
        if self.filename_stdout:
            descriptions.append('stdout > file:' + self.filename_stdout)
        if self.filename_stderr:
            descriptions.append('stderr > file:' + self.filename_stderr)
        return ', '.join(descriptions)

    def process_init(self):
        if self.filename_stdout:
            mode = 'wb'
            if self.handle_stdout:
                # append to previous file if this is a reinvocation
                mode = mode.replace('w', 'a')
            if not self.handle_stdout or self.handle_stdout.closed:
                self.handle_stdout = open(self.filename_stdout, mode)
        if self.filename_stderr:
            mode = 'wb'
            if self.handle_stderr:
                # append to previous file if this is a reinvocation
                mode = mode.replace('w', 'a')
            if not self.handle_stderr or self.handle_stderr.closed:
                self.handle_stderr = open(self.filename_stderr, mode)
        if self.filename_stderr is False:
            self.handle_stderr = self.handle_stdout

    def support_stderr2stdout(self):
        return self.filename_stderr is False

    def on_stdout_lines(self, lines):
        if self.handle_stdout:
            self.handle_stdout.write(lines)
            self.handle_stdout.flush()

    def on_stderr_lines(self, lines):
        if self.handle_stderr:
            self.handle_stderr.write(lines)
            self.handle_stderr.flush()

    def process_cleanup(self):
        if self.handle_stdout:
            self.handle_stdout.close()
        if self.handle_stderr:
            if not self.handle_stderr.closed:
                self.handle_stderr.close()
