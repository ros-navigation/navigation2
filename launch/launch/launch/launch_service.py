# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the LaunchService class."""

import asyncio
import atexit
import collections
import logging
import signal
import threading
import traceback
from typing import Iterable
from typing import List  # noqa: F401
from typing import Optional
from typing import Set  # noqa: F401
from typing import Text
from typing import Tuple  # noqa: F401

import osrf_pycommon.process_utils

from .event import Event
from .event_handlers import OnIncludeLaunchDescription
from .event_handlers import OnShutdown
from .events import IncludeLaunchDescription
from .events import Shutdown
from .launch_context import LaunchContext
from .launch_description import LaunchDescription
from .launch_description_entity import LaunchDescriptionEntity
from .some_actions_type import SomeActionsType
from .utilities import install_signal_handlers
from .utilities import on_sigint
from .utilities import on_sigquit
from .utilities import on_sigterm
from .utilities import visit_all_entities_and_collect_futures

_logger = logging.getLogger('launch.LaunchService')
_g_loops_used = set()  # type: Set


# This atexit handler ensures all the loops are closed at exit.
# This is only really required pre-3.6, see:
#   https://github.com/ros2/launch/issues/84
#   https://bugs.python.org/issue23548
@atexit.register
def close_loop():
    global _g_loops_used
    for loop in _g_loops_used:
        if not loop.is_closed():
            loop.close()


class LaunchService:
    """Service that manages the event loop and runtime for launched system."""

    def __init__(
        self,
        *,
        argv: Optional[Iterable[Text]] = None,
        debug: bool = False
    ) -> None:
        """
        Constructor.

        If called outside of the main-thread before the function
        :func:`launch.utilities.install_signal_handlers()` has been called,
        a ValueError can be raised, as setting signal handlers cannot be done
        outside of the main-thread.

        :param: argv stored in the context for access by the entities, None results in []
        :param: debug if True (not default), asyncio the logger are seutp for debug
        """
        # Install signal handlers if not already installed, will raise if not
        # in main-thread, call manually in main-thread to avoid this.
        install_signal_handlers()

        self.__argv = argv if argv is not None else []

        # Setup logging and debugging.
        logging.basicConfig(
            level=logging.INFO,
            format='[%(levelname)s] [%(name)s]: %(msg)s',
        )
        self.__debug = debug
        if self.__debug:
            logging.getLogger('launch').setLevel(logging.DEBUG)
        else:
            logging.getLogger('launch').setLevel(logging.INFO)

        # Setup context and register a built-in event handler for bootstrapping.
        self.__context = LaunchContext(argv=self.__argv)
        self.__context.register_event_handler(OnIncludeLaunchDescription())
        self.__context.register_event_handler(OnShutdown(on_shutdown=self.__on_shutdown))

        # Setup storage for state.
        self._entity_future_pairs = \
            []  # type: List[Tuple[LaunchDescriptionEntity, asyncio.Future]]

        # Used to prevent run() being called from multiple threads.
        self.__running_lock = threading.Lock()
        self.__running = False

        # Used to allow asynchronous use of self.__loop_from_run_thread without
        # it being set to None by run() as it exits.
        self.__loop_from_run_thread_lock = threading.RLock()
        self.__loop_from_run_thread = None

        # Used to indicate when shutdown() has been called.
        self.__shutting_down = False
        self.__shutdown_when_idle = False

        # Used to keep track of whether or not there were unexpected exceptions.
        self.__return_code = 0

    def emit_event(self, event: Event) -> None:
        """
        Emit an event synchronously and thread-safely.

        If the LaunchService is not running, the event is queued until it is.
        """
        with self.__loop_from_run_thread_lock:
            if self.__loop_from_run_thread is not None:
                # loop is in use, asynchronously emit the event
                future = asyncio.run_coroutine_threadsafe(
                    self.__context.emit_event(event),
                    self.__loop_from_run_thread
                )
                future.result()
            else:
                # loop is not in use, synchronously emit the event, and it will be processed later
                self.__context.emit_event_sync(event)

    def include_launch_description(self, launch_description: LaunchDescription) -> None:
        """
        Evaluate a given LaunchDescription and visits all of its entities.

        This method is thread-safe.
        """
        self.emit_event(IncludeLaunchDescription(launch_description))

    def _prune_and_count_entity_future_pairs(self):
        needs_prune = False
        for pair in self._entity_future_pairs:
            if pair[1].done():
                needs_prune = True
        if needs_prune:
            self._entity_future_pairs = \
                [pair for pair in self._entity_future_pairs if not pair[1].done()]
        return len(self._entity_future_pairs)

    def _prune_and_count_context_completion_futures(self):
        needs_prune = False
        for future in self.__context._completion_futures:
            if future.done():
                needs_prune = True
        if needs_prune:
            self.__context._completion_futures = \
                [f for f in self.__context._completion_futures if not f.done()]
        return len(self.__context._completion_futures)

    def _is_idle(self):
        number_of_entity_future_pairs = self._prune_and_count_entity_future_pairs()
        number_of_entity_future_pairs += self._prune_and_count_context_completion_futures()
        return number_of_entity_future_pairs == 0 and self.__context._event_queue.empty()

    async def _process_one_event(self) -> None:
        next_event = await self.__context._event_queue.get()
        await self.__process_event(next_event)

    async def __process_event(self, event: Event) -> None:
        _logger.debug("processing event: '{}'".format(event))
        for event_handler in tuple(self.__context._event_handlers):
            if event_handler.matches(event):
                _logger.debug(
                    "processing event: '{}' ✓ '{}'".format(event, event_handler))
                self.__context._push_locals()
                entities = event_handler.handle(event, self.__context)
                entities = entities if isinstance(entities, collections.Iterable) else (entities,)
                for entity in [e for e in entities if e is not None]:
                    from .utilities import is_a_subclass
                    if not is_a_subclass(entity, LaunchDescriptionEntity):
                        raise RuntimeError(
                            "expected a LaunchDescriptionEntity from event_handler, got '{}'"
                            .format(entity)
                        )
                    self._entity_future_pairs.extend(
                        visit_all_entities_and_collect_futures(entity, self.__context))
                self.__context._pop_locals()
            else:
                pass
                # Keep this commented for now, since it's very chatty.
                # _logger.debug(
                #     "processing event: '{}' x '{}'".format(event, event_handler))

    async def __run_loop(self) -> None:
        while True:
            # Check if we're idle, i.e. no on-going entities (actions) or events in the queue
            is_idle = self._is_idle()  # self._entity_future_pairs is pruned here
            if not self.__shutting_down and self.__shutdown_when_idle and is_idle:
                self._shutdown(reason='idle', due_to_sigint=False)

            if self.__loop_from_run_thread is None:
                raise RuntimeError('__loop_from_run_thread unexpectedly None')
            process_one_event_task = self.__loop_from_run_thread.create_task(
                self._process_one_event())
            if self.__shutting_down:
                # If shutting down and idle then we're done.
                if is_idle:
                    process_one_event_task.cancel()
                    return
                else:
                    entity_futures = [pair[1] for pair in self._entity_future_pairs]
                    entity_futures.append(process_one_event_task)
                    entity_futures.extend(self.__context._completion_futures)
                    done = set()  # type: Set[asyncio.Future]
                    while not done:
                        done, pending = await asyncio.wait(
                            entity_futures,
                            loop=self.__loop_from_run_thread,
                            timeout=1.0,
                            return_when=asyncio.FIRST_COMPLETED)
                        if not done:
                            _logger.debug('still waiting on futures: {}'.format(entity_futures))
            else:
                await process_one_event_task

    def run(self, *, shutdown_when_idle=True) -> int:
        """
        Start the event loop and visit all entities of all included LaunchDescriptions.

        This should only ever be run from a single thread.

        :param: shutdown_when_idle if True (default), the service will shutdown when idle
        """
        # Make sure this has not been called in multiple threads.
        with self.__running_lock:
            if self.__running:
                raise RuntimeError(
                    'LaunchService.run() called from multiple threads concurrently.')
            self.__running = True

        self.__return_code = 0  # reset the return_code for this run()
        self.__shutdown_when_idle = shutdown_when_idle

        # Acquire the lock and initialize the asyncio loop.
        with self.__loop_from_run_thread_lock:
            self.__loop_from_run_thread = osrf_pycommon.process_utils.get_loop()
            if self.__loop_from_run_thread is None:
                raise RuntimeError('__loop_from_run_thread unexpectedly None')
            global _g_loops_used
            _g_loops_used.add(self.__loop_from_run_thread)
            if self.__debug:
                self.__loop_from_run_thread.set_debug(True)

            # Setup the exception handler to make sure we return non-0 when there are errors.
            def exception_handler(loop, context):
                self.__return_code = 1
                return loop.default_exception_handler(context)
            self.__loop_from_run_thread.set_exception_handler(exception_handler)

            # Set the asyncio loop for the context.
            self.__context._set_asyncio_loop(self.__loop_from_run_thread)
            # Recreate the event queue to ensure the same event loop is being used.
            new_queue = asyncio.Queue(loop=self.__loop_from_run_thread)
            while True:
                try:
                    new_queue.put_nowait(self.__context._event_queue.get_nowait())
                except asyncio.QueueEmpty:
                    break
            self.__context._event_queue = new_queue

        # Run the asyncio loop over the main coroutine that processes events.
        try:
            sigint_received = False
            run_loop_task = self.__loop_from_run_thread.create_task(self.__run_loop())

            # Setup custom signal hanlders for SIGINT, SIGQUIT, and SIGTERM.
            def _on_sigint(signum, frame):
                # Ignore additional interrupts until we finish processing this one.
                prev_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
                if prev_handler is signal.SIG_IGN:
                    # This function has been called re-entrantly.
                    return

                nonlocal sigint_received
                base_msg = 'user interrupted with ctrl-c (SIGINT)'
                if not sigint_received:
                    _logger.warn(base_msg)
                    self._shutdown(reason='ctrl-c (SIGINT)', due_to_sigint=True)
                    sigint_received = True
                else:
                    _logger.warn('{} again, ignoring...'.format(base_msg))

                signal.signal(signal.SIGINT, prev_handler)

            def _on_sigterm(signum, frame):
                # Ignore additional signals until we finish processing this one.
                prev_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
                if prev_handler is signal.SIG_IGN:
                    # This function has been called re-entrantly.
                    return

                # TODO(wjwwood): try to terminate running subprocesses before exiting.
                _logger.error('using SIGTERM or SIGQUIT can result in orphaned processes')
                _logger.error('make sure no processes launched are still running')
                nonlocal run_loop_task
                self.__loop_from_run_thread.call_soon_threadsafe(run_loop_task.cancel)

                signal.signal(signal.SIGTERM, prev_handler)

            def _on_sigquit(signum, frame):
                # Ignore additional signals until we finish processing this one.
                prev_handler = signal.signal(signal.SIGQUIT, signal.SIG_IGN)
                if prev_handler is signal.SIG_IGN:
                    # This function has been called re-entrantly.
                    return

                _logger.error('user interrupted with ctrl-\\ (SIGQUIT), terminating...')
                _on_sigterm(signum, frame)

                signal.signal(signal.SIGQUIT, prev_handler)

            on_sigint(_on_sigint)
            on_sigterm(_on_sigterm)
            on_sigquit(_on_sigquit)

            while not run_loop_task.done():
                try:
                    self.__loop_from_run_thread.run_until_complete(run_loop_task)
                except KeyboardInterrupt:
                    pass
                except asyncio.CancelledError:
                    _logger.error('asyncio run loop was canceled')
                except Exception as exc:
                    msg = 'Caught exception in launch (see debug for traceback): {}'.format(exc)
                    _logger.debug(traceback.format_exc())
                    _logger.error(msg)
                    self.__return_code = 1
                    self._shutdown(reason=msg, due_to_sigint=False)
                    # restart run loop to let it shutdown properly
                    run_loop_task = self.__loop_from_run_thread.create_task(self.__run_loop())
        finally:
            # No matter what happens, unset the loop and set running to false.
            with self.__loop_from_run_thread_lock:
                self.__shutting_down = False
                self.__loop_from_run_thread = None
                self.__context._set_asyncio_loop(None)

            # Unset the signal handlers while not running.
            on_sigint(None)
            on_sigterm(None)
            on_sigquit(None)

            with self.__running_lock:
                self.__running = False

        return self.__return_code

    def __on_shutdown(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        self.__shutting_down = True
        return None

    def _shutdown(self, *, reason, due_to_sigint):
        # Assumption is that this method is only called when running.
        if not self.__shutting_down:
            shutdown_event = Shutdown(reason=reason, due_to_sigint=due_to_sigint)
            asyncio_event_loop = None
            try:
                asyncio_event_loop = asyncio.get_event_loop()
            except (RuntimeError, AssertionError):
                # If no event loop is set for this thread, asyncio will raise an exception.
                # The exception type depends on the version of Python, so just catch both.
                pass
            if self.__loop_from_run_thread == asyncio_event_loop:
                # If in the thread of the loop.
                self.__loop_from_run_thread.create_task(self.__context.emit_event(shutdown_event))
            else:
                # Otherwise in a different thread, so use the thread-safe method.
                self.emit_event(shutdown_event)
        self.__shutting_down = True
        self.__context._set_is_shutdown(True)

    def shutdown(self) -> None:
        """
        Shutdown all on-going activities and then stop the asyncio run loop.

        This will cause LaunchService.run() to eventually exit.

        Does nothing if LaunchService.run() is not running in another thread.

        This method is thread-safe.
        """
        with self.__loop_from_run_thread_lock:
            if self.__loop_from_run_thread is not None:
                self._shutdown(reason='LaunchService.shutdown() called', due_to_sigint=False)
