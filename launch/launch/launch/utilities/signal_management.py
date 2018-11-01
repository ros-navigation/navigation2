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

"""Module for the signal management functionality."""

import logging
import platform
import signal
import threading

__signal_handlers_installed_lock = threading.Lock()
__signal_handlers_installed = False
__custom_sigint_handler = None
__custom_sigquit_handler = None
__custom_sigterm_handler = None

_logger = logging.getLogger('launch.utilities.signal_management')


def on_sigint(handler):
    """
    Set the signal handler to be called on SIGINT.

    Pass None for no custom handler.
    Note that if a custom handler is set (anything other than None is given),
    then KeyboardInterrupt is caught and ignored around the original signal
    handler (the once captured when install_signal_handlers() was called).
    Passing None for handler will undo this behavior.

    install_signal_handlers() must have been called in the main thread before.
    It is called automatically by the constructor of `launch.LaunchService`.
    """
    global __custom_sigint_handler
    if handler is not None and not callable(handler):
        raise ValueError('handler must be callable or None')
    __custom_sigint_handler = handler


def on_sigquit(handler):
    """
    Set the signal handler to be called on SIGQUIT.

    Note Windows does not have SIGQUIT, so it can be set with this function,
    but the handler will not be called.

    Pass None for no custom handler.

    install_signal_handlers() must have been called in the main thread before.
    It is called automatically by the constructor of `launch.LaunchService`.
    """
    global __custom_sigquit_handler
    if handler is not None and not callable(handler):
        raise ValueError('handler must be callable or None')
    __custom_sigquit_handler = handler


def on_sigterm(handler):
    """
    Set the signal handler to be called on SIGTERM.

    Pass None for no custom handler.

    install_signal_handlers() must have been called in the main thread before.
    It is called automatically by the constructor of `launch.LaunchService`.
    """
    global __custom_sigterm_handler
    if handler is not None and not callable(handler):
        raise ValueError('handler must be callable or None')
    __custom_sigterm_handler = handler


def install_signal_handlers():
    """
    Install custom signal handlers so that hooks can be setup from other threads.

    Calling this multiple times does not fail, but the signals are only
    installed once.

    If called outside of the main-thread, a ValueError is raised, see:
    https://docs.python.org/3.6/library/signal.html#signal.signal

    Also, if you register your own signal handlers after calling this function,
    then you should store and forward to the existing signal handlers, because
    otherwise the signal handlers registered by on_sigint(), on_sigquit, etc.
    will not be run.
    And the signal handlers registered with those functions are used to
    gracefully exit the LaunchService when signaled (at least), and without
    them it may not behave correctly.

    If you register signal handlers before calling this function, then your
    signal handler will automatically be called by the signal handlers in this
    thread.
    """
    global __signal_handlers_installed_lock, __signal_handlers_installed
    with __signal_handlers_installed_lock:
        if __signal_handlers_installed:
            return
        __signal_handlers_installed = True

    global __custom_sigint_handler, __custom_sigquit_handler, __custom_sigterm_handler

    __original_sigint_handler = signal.getsignal(signal.SIGINT)
    __original_sigterm_handler = signal.getsignal(signal.SIGTERM)

    def __on_sigint(signum, frame):
        if callable(__custom_sigint_handler):
            __custom_sigint_handler(signum, frame)
        if callable(__original_sigint_handler):
            __original_sigint_handler(signum, frame)

    if platform.system() != 'Windows':
        # Windows does not support SIGQUIT
        __original_sigquit_handler = signal.getsignal(signal.SIGQUIT)

        def __on_sigquit(signum, frame):
            if callable(__custom_sigquit_handler):
                __custom_sigquit_handler(signum, frame)
            if callable(__original_sigquit_handler):
                __original_sigquit_handler(signum, frame)

    def __on_sigterm(signum, frame):
        if callable(__custom_sigterm_handler):
            __custom_sigterm_handler(signum, frame)
        if callable(__original_sigterm_handler):
            __original_sigterm_handler(signum, frame)

    # signals must be registered in the main thread, but print a nicer message if we're not there
    try:
        signal.signal(signal.SIGINT, __on_sigint)
        signal.signal(signal.SIGTERM, __on_sigterm)
        if platform.system() != 'Windows':
            # Windows does not support SIGQUIT
            signal.signal(signal.SIGQUIT, __on_sigquit)
    except ValueError:
        _logger.error("failed to set signal handlers in 'launch.utilities.signal_management.py'")
        _logger.error('this function must be called in the main thread')
        raise
