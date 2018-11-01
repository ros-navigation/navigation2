from importlib.machinery import SourceFileLoader
import io
import os
import re
import signal
import sys

import ament_index_python
from launch.legacy.output_handler import LineOutput


class UnmatchedOutputError(BaseException):
    pass


class InMemoryHandler(LineOutput):
    """
    Aggregate data from standard output.

    :param name: Name of the process being tested.
    :param launch_descriptor: :py:obj:`LaunchDescriptor` object that contains the processes in the
        test.
    :param expected_lines: A list of lines to match the output literally or a regular expression
        that will only need one line to match, instead of the entire output.
    :param regex_match: If true, treat the expected_lines as a regular expression in match
        accordingly.
    :param filtered_prefixes: A list of byte strings representing prefixes that will cause output
        lines to be ignored if they start with one of the prefixes. By default lines starting with
        the process ID (`b'pid'`) and return code (`b'rc'`) will be ignored.
    :param filtered_patterns: A list of byte strings representing regexes that will cause output
        lines to be ignored if they match one of the regexes.
    :param filtered_rmw_implementation: RMW implementation for which the output will be ignored
        in addition to the `filtered_prefixes`/`filtered_patterns`.
    :param exit_on_match: If True, then when its output is matched, this handler
        will terminate; otherwise it will simply keep track of the match.
    :raises: :py:class:`UnmatchedOutputError` if :py:meth:`check` does not find that the output
        matches as expected.
    :raises: :exc:`LookupError` if the `rmw_output_filter` of the `filtered_rmw_implementation`
        cannot be found.
    :raises: :exc:`IOError` if the `rmw_output_filter` of the `filtered_rmw_implementation`
        cannot be opened.
    """

    def __init__(
        self, name, launch_descriptor, expected_lines, regex_match=False,
        filtered_prefixes=None, filtered_patterns=None, filtered_rmw_implementation=None,
        exit_on_match=True,
    ):
        super(LineOutput, self).__init__()
        self.filtered_prefixes = filtered_prefixes or get_default_filtered_prefixes()
        self.filtered_patterns = filtered_patterns or get_default_filtered_patterns()

        if filtered_rmw_implementation:
            self.filtered_prefixes.extend(
                get_rmw_output_filter(filtered_rmw_implementation, 'prefixes'))
            self.filtered_patterns.extend(
                get_rmw_output_filter(filtered_rmw_implementation, 'patterns'))

        self.name = name
        self.launch_descriptor = launch_descriptor
        self.expected_lines = expected_lines
        self.left_over_stdout = b''
        self.left_over_stderr = b''
        self.stdout_data = io.BytesIO()
        self.stderr_data = io.BytesIO()
        self.regex_match = regex_match
        self.exit_on_match = exit_on_match
        if not self.regex_match:
            self.unmatched = list(self.expected_lines)
        else:
            self.unmatched = []
            for l in self.expected_lines:
                try:
                    regex = re.compile(l)
                except Exception as e:
                    print('Failed to compile regex [%s]: %s' % (l, e), file=sys.stderr)
                    raise
                self.unmatched.append(regex)

    def on_stdout_lines(self, lines):
        if not self.unmatched:
            return

        for line in lines.splitlines():
            # Filter out stdout that comes from underlying DDS implementation
            # Note: we do not currently support matching filters across multiple stdout lines.
            if any(line.startswith(prefix) for prefix in self.filtered_prefixes):
                continue
            if any(re.match(pattern, line) for pattern in self.filtered_patterns):
                continue
            self.stdout_data.write(line + b'\n')

        output_lines = self.stdout_data.getvalue()
        if not self.regex_match:
            output_lines = output_lines.splitlines()
        for unmatched in list(self.unmatched):
            if not self.regex_match:
                if unmatched in output_lines:
                    self.unmatched.remove(unmatched)
            else:
                if unmatched.search(self.stdout_data.getvalue()):
                    self.unmatched.remove(unmatched)

        if not self.unmatched and self.exit_on_match:
            # We matched and we're in charge; shut myself down
            for td in self.launch_descriptor.task_descriptors:
                if td.name == self.name:
                    if os.name != 'nt':
                        td.task_state.signals_received.append(signal.SIGINT)
                        td.transport.send_signal(signal.SIGINT)
                    else:
                        td.terminate()
                    return

    def on_stderr_lines(self, lines):
        self.stderr_data.write(lines)

    def get_description(self):
        return 'InMemoryHandler: ' + self.name

    def check(self):
        output_lines = self.stdout_data.getvalue().splitlines()
        if self.unmatched:
            raise UnmatchedOutputError(
                'Example output (%r) does not match expected output (%r)' %
                (output_lines, self.expected_lines))


def get_default_filtered_prefixes():
    return [
        b'pid', b'rc',
    ]


def get_default_filtered_patterns():
    return []


def get_rmw_output_filter(rmw_implementation, filter_type):
    supported_filter_types = ['prefixes', 'patterns']
    if filter_type not in supported_filter_types:
        raise TypeError(
            'Unsupported filter_type "{0}". Supported types: {1}'
            .format(filter_type, supported_filter_types))
    resource_name = 'rmw_output_' + filter_type
    prefix_with_resource = ament_index_python.has_resource(
        resource_name, rmw_implementation)
    if not prefix_with_resource:
        return []

    # Treat each line of the resource as an independent filter.
    rmw_output_filter, _ = ament_index_python.get_resource(resource_name, rmw_implementation)
    return [str.encode(l) for l in rmw_output_filter.splitlines()]


def create_handler(
    name, launch_descriptor, output_file, exit_on_match=True, filtered_prefixes=None,
    filtered_patterns=None, filtered_rmw_implementation=None
):
    literal_file = output_file + '.txt'
    if os.path.isfile(literal_file):
        with open(literal_file, 'rb') as f:
            expected_output = f.read().splitlines()
        return InMemoryHandler(
            name, launch_descriptor, expected_output, regex_match=False,
            exit_on_match=exit_on_match,
            filtered_prefixes=filtered_prefixes, filtered_patterns=filtered_patterns,
            filtered_rmw_implementation=filtered_rmw_implementation)
    regex_file = output_file + '.regex'
    if os.path.isfile(regex_file):
        with open(regex_file, 'rb') as f:
            expected_output = f.read().splitlines()
        return InMemoryHandler(
            name, launch_descriptor, expected_output, regex_match=True,
            exit_on_match=exit_on_match,
            filtered_prefixes=filtered_prefixes, filtered_patterns=filtered_patterns,
            filtered_rmw_implementation=filtered_rmw_implementation)
    py_file = output_file + '.py'
    if os.path.isfile(py_file):
        checker_module = SourceFileLoader(
            'checker_module', py_file).load_module()
        return checker_module.CheckerHandler(name, launch_descriptor)
