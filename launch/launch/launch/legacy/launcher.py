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
from collections import OrderedDict
import os
import signal
import sys
import threading
import time

from launch.legacy.exit_handler import ExitHandlerContext
from launch.legacy.launch import LaunchState
from launch.legacy.protocol import SubprocessProtocol
from launch.legacy.task import TaskState


class _TaskException(Exception):

    def __init__(self, task_descriptor_index, exception):
        self.task_descriptor_index = task_descriptor_index
        self.exception = exception


class DefaultLauncher:

    def __init__(self, name_prefix='', sigint_timeout=10):
        self.name_prefix = name_prefix
        self.sigint_timeout = sigint_timeout
        self.task_descriptors = []
        self.print_mutex = threading.Lock()
        self.loop = None
        self.loop_lock = threading.Lock()
        self.interrupt_future = asyncio.Future()
        self.launch_complete = threading.Event()
        self.processes_spawned = threading.Event()

    def add_launch_descriptor(self, launch_descriptor):
        for task_descriptor in launch_descriptor.task_descriptors:
            # automatic naming if not specified
            if task_descriptor.name is None:
                name = str(len(self.task_descriptors))
                if name in [p.name for p in self.task_descriptors]:
                    raise RuntimeError("Process name '%s' already used" % name)
                task_descriptor.name = name

            self.task_descriptors.append(task_descriptor)

    def interrupt_launch(self):
        with self.loop_lock:
            if self.loop is not None:
                self.loop.call_soon_threadsafe(self.interrupt_launch_non_threadsafe)

    def interrupt_launch_non_threadsafe(self):
        self.interrupt_future.set_result(True)

    def wait_on_launch_to_finish(self, timeout):
        return self.launch_complete.wait(timeout)

    def is_launch_running(self):
        return not self.launch_complete.is_set()

    def wait_on_processes_to_spawn(self, timeout):
        return self.processes_spawned.wait(timeout)

    def are_processes_spawned(self):
        return self.processes_spawned.is_set()

    def launch(self):
        with self.loop_lock:
            if os.name == 'nt':
                # Windows needs a custom event loop to use subprocess transport
                self.loop = asyncio.ProactorEventLoop()
            else:
                self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
        loop = self.loop
        try:
            generator = self._run()
            returncode = loop.run_until_complete(generator)
        except _TaskException as e:
            print(
                'Failed to execute command: ' +
                ' '.join(self.task_descriptors[e.task_descriptor_index].cmd), file=sys.stderr)
            raise e.exception
        except KeyboardInterrupt:
            # ignore further keyboard interrupts to cleanly shutdown
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            self.interrupt_launch_non_threadsafe()
            loop.run_forever()
            returncode = 1
        loop.close()
        with self.loop_lock:
            self.loop = None
        if os.name != 'nt':
            # the watcher must be reset otherwise a repeated invocation fails inside asyncio
            asyncio.get_event_loop_policy().set_child_watcher(None)

        return returncode

    async def _run(self):
        self.interrupt_future = asyncio.Future()
        self.launch_complete.clear()
        self.processes_spawned.clear()
        launch_state = LaunchState()
        for p in self.task_descriptors:
            p.task_state = TaskState()

        # start all processes and collect their exit futures
        all_futures = OrderedDict()
        for index, p in enumerate(self.task_descriptors):
            if 'output_handler' in dir(p):
                p.output_handler.set_print_mutex(self.print_mutex)
                p.output_handler.set_line_prefix('[%s] ' % p.name)

            if 'protocol' in dir(p):
                try:
                    await self._spawn_process(index)
                except Exception:
                    # clean up already spawned processes
                    await self._terminate_processes(launch_state, all_futures)
                    self.launch_complete.set()
                    return 1
                all_futures[p.protocol.exit_future] = index
            else:
                future = asyncio.ensure_future(p.coroutine)
                all_futures[future] = index

        # the processes are not guaranteed to be running yet, but at least you
        # can say that it was not possible for all of them to be running before
        # this point
        self.processes_spawned.set()

        while True:
            # skip if no more processes to run
            if not all_futures:
                break

            # wait for any process to finish
            kwargs = {
                'return_when': asyncio.FIRST_COMPLETED,
            }
            # when the event loop run does not run in the main thread
            # wake up frequently and check if any subprocess has exited
            if not isinstance(threading.current_thread(), threading._MainThread):
                kwargs['timeout'] = 0.5
            await asyncio.wait(list(all_futures.keys()) + [self.interrupt_future], **kwargs)

            # if asynchronously interrupted, stop looping and shutdown
            if self.interrupt_future.done():
                break

            # when the event loop run does not run in the main thread
            # use custom logic to detect that subprocesses have exited
            if not isinstance(threading.current_thread(), threading._MainThread):
                self.check_for_exited_subprocesses(all_futures)

            # collect done futures
            done_futures = [f for f in all_futures.keys() if f.done()]

            # collect return code
            restart_indices = []
            for future in done_futures:
                index = all_futures[future]
                p = self.task_descriptors[index]

                # collect return code / exception from coroutine
                if 'coroutine' in dir(p):
                    exp = future.exception()
                    if exp:
                        p.task_state.exception = exp
                        p.task_state.returncode = 1
                        self._print_process_stacktrace(p.name, future, exp)
                    else:
                        result = future.result()
                        p.task_state.returncode = result
                    self._process_message(p, 'rc ' + str(p.task_state.returncode))

                # close transport
                if 'protocol' in dir(p):
                    self._close_process(p)

                # remove future
                del all_futures[future]

                # call exit handler of done descriptors
                context = ExitHandlerContext(launch_state, p.task_state)
                p.exit_handler(context)
                if p.task_state.restart:
                    restart_indices.append(index)

            if launch_state.teardown:
                with self.print_mutex:
                    print('() tear down')
                break

            # restart processes if requested
            for index in restart_indices:
                p = self.task_descriptors[index]
                if 'protocol' in dir(p):
                    p.task_state.restart_count += 1
                    await self._spawn_process(index)
                    all_futures[p.protocol.exit_future] = index
        # end while True

        await self._terminate_processes(launch_state, all_futures)

        if launch_state.returncode is None:
            launch_state.returncode = 0
        self.launch_complete.set()
        self.loop.stop()
        return launch_state.returncode

    async def _terminate_processes(self, launch_state, all_futures):
        # terminate all remaining processes
        if all_futures:

            # sending SIGINT to subprocess transport is not supported on Windows
            # https://groups.google.com/forum/#!topic/python-tulip/pr9fgX8Vh-A
            if os.name != 'nt':
                # sending SIGINT to remaining processes
                for index in all_futures.values():
                    p = self.task_descriptors[index]
                    if 'transport' in dir(p):
                        self._process_message(p, 'signal SIGINT')
                        try:
                            p.transport.send_signal(signal.SIGINT)
                            p.task_state.signals_received.append(signal.SIGINT)
                        except ProcessLookupError:
                            pass

                if isinstance(threading.current_thread(), threading._MainThread):
                    # if in the main thread, just wait
                    await asyncio.wait(all_futures.keys(), timeout=self.sigint_timeout)
                else:
                    # if not in the main thread, wake up periodically to check SIGINT status
                    start = time.time()
                    short_timeout = self.sigint_timeout / 10.0
                    while time.time() - start < self.sigint_timeout:
                        # each loop check for recently exited subprocesses
                        self.check_for_exited_subprocesses(all_futures)
                        if not [fut for fut in all_futures.keys() if not fut.done()]:
                            # if all of the futures are done, stop waiting
                            break
                        await asyncio.wait(all_futures.keys(), timeout=short_timeout)

            # cancel coroutines
            for future, index in all_futures.items():
                p = self.task_descriptors[index]
                if 'coroutine' in dir(p):
                    if not future.done():
                        self._process_message(p, 'cancel coroutine')
                        future.cancel()

            # sending SIGTERM to remaining processes
            for index in all_futures.values():
                p = self.task_descriptors[index]
                if 'protocol' in dir(p):
                    if not p.protocol.exit_future.done():
                        self._process_message(p, 'signal SIGTERM')
                        try:
                            p.transport.send_signal(signal.SIGTERM)
                            p.task_state.signals_received.append(signal.SIGTERM)
                        except ProcessLookupError:
                            pass

            kwargs = {}
            if not isinstance(threading.current_thread(), threading._MainThread):
                # wake up periodically if we are not in the main thread
                kwargs['timeout'] = 0.5
            pending = None
            while pending is None or pending:
                # when the event loop run does not run in the main thread
                # use custom logic to detect that subprocesses have exited
                if not isinstance(threading.current_thread(), threading._MainThread):
                    self.check_for_exited_subprocesses(all_futures)
                # wait for futures to be complete
                _, pending = await asyncio.wait(all_futures.keys(), **kwargs)

            # close all remaining processes
            for index in all_futures.values():
                p = self.task_descriptors[index]
                if 'transport' in dir(p):
                    self._close_process(p)

            # call exit handler of remaining descriptors
            for future, index in all_futures.items():
                p = self.task_descriptors[index]

                # collect return code / exception from coroutine
                if 'coroutine' in dir(p):
                    try:
                        exp = future.exception()
                        if exp:
                            p.task_state.exception = exp
                            p.task_state.returncode = 1
                            self._print_process_stacktrace(p.name, future, exp)
                        else:
                            result = future.result()
                            p.task_state.returncode = result
                    except asyncio.CancelledError:
                        p.task_state.returncode = 0
                    self._process_message(p, 'rc ' + str(p.task_state.returncode))

                context = ExitHandlerContext(launch_state, p.task_state)
                p.exit_handler(context)

    def check_for_exited_subprocesses(self, all_futures):
        for index, p in enumerate(self.task_descriptors):
            # only consider not yet done tasks
            if index not in all_futures.values():
                continue
            # only subprocesses need special handling
            if 'transport' not in dir(p):
                continue
            # transport.get_pid() sometimes failed due to transport._proc being None
            proc = p.transport.get_extra_info('subprocess')
            if os.name != 'nt':
                # wait non-blocking on pid
                pid = proc.pid
                try:
                    pid, pid_rc = os.waitpid(pid, os.WNOHANG)
                except ChildProcessError:
                    continue
                if pid == 0:
                    # subprocess is still running
                    continue
                p.returncode = pid_rc
            else:
                # use subprocess return code, only works on Windows
                if proc.returncode is None:
                    continue
                p.returncode = proc.returncode

            # trigger asyncio internal process exit callback
            p.transport._process_exited(p.returncode)

    async def _spawn_process(self, index):
        p = self.task_descriptors[index]
        p.output_handler.process_init()
        kwargs = {}
        if p.output_handler.support_stderr2stdout():
            kwargs['stderr'] = asyncio.subprocess.STDOUT
        if p.env is not None:
            kwargs['env'] = p.env
        loop = asyncio.get_event_loop()
        try:
            transport, protocol = await loop.subprocess_exec(
                lambda: SubprocessProtocol(p.output_handler),
                *p.cmd,
                **kwargs)
        except Exception as e:
            self._process_message(
                p, 'Failed to spawn command %s: %s' % (p.cmd, e))
            raise
        p.transport = transport
        p.protocol = protocol

        output_handler_description = p.output_handler.get_description()
        if 'stderr' in kwargs:
            output_handler_description = 'stderr > stdout, ' + output_handler_description

        self._process_message(
            p, 'pid %d: %s (%s)' % (transport.get_pid(), p.cmd, output_handler_description))

    def _close_process(self, process_descriptor):
        p = process_descriptor
        p.transport.close()
        p.task_state.returncode = p.transport.get_returncode()
        self._process_message(p, 'rc ' + str(p.task_state.returncode))
        p.output_handler.process_cleanup()

    def _process_message(self, process_descriptor, message):
        p = process_descriptor

        with self.print_mutex:
            print('(%s)' % p.name, message)
        lines = (message + '\n').encode()
        if 'output_handler' in dir(p):
            p.output_handler.on_message_received(lines)

    def _print_process_stacktrace(self, name, future, exception):
        # print traceback with "standard" format
        with self.print_mutex:
            print('(%s)' % name, 'Traceback (most recent call last):',
                  file=sys.stderr)
            for frame in future.get_stack():
                filename = frame.f_code.co_filename
                print('(%s)' % name, '  File "%s", line %d, in %s' %
                      (filename, frame.f_lineno, frame.f_code.co_name),
                      file=sys.stderr)
                import linecache
                linecache.checkcache(filename)
                line = linecache.getline(filename, frame.f_lineno, frame.f_globals)
                print('(%s)' % name, '    ' + line.strip(), file=sys.stderr)
            print('(%s) %s: %s' % (name, type(exception).__name__, str(exception)),
                  file=sys.stderr)
