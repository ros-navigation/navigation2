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

import os


class ExitHandlerContext:
    """The context which is passed to an exit handler function."""

    def __init__(self, launch_state, task_state):
        self.launch_state = launch_state
        self.task_state = task_state


def default_exit_handler(context, ignore_returncode=False):
    """
    Trigger teardown of launch.

    Use the returncode of the task for the launch if the launch was not already tearing down.

    @param ignore_returncode: If True, then ignore process return code.
    """
    # trigger tear down if not already tearing down
    if not context.launch_state.teardown:
        context.launch_state.teardown = True

    # set launch return code if not already set
    if (
        not context.launch_state.returncode and
        # since Python < 3.5 on Windows does not support signaling SIGINT to the subprocesses
        # we can't expect them to shutdown cleanly, therefore we ignore their return code
        (os.name != 'nt' or not context.launch_state.teardown) and
        # allow force ignoring
        not ignore_returncode
    ):
        try:
            rc = int(context.task_state.returncode)
        except (TypeError, ValueError):
            rc = 1 if bool(context.task_state.returncode) else 0
        context.launch_state.returncode = rc


def ignore_exit_handler(context):
    """Continue the launch and don't affect the returncode of the launch."""
    pass


def restart_exit_handler(context):
    """Request restart of the task."""
    context.task_state.restart = True


def primary_exit_handler(context):
    """
    Trigger teardown of launch and if teardown already in place set non-zero return code.

    Same as default exit handler but if teardown was triggered by another task
    ensure that the returncode is non-zero.
    """
    if context.launch_state.teardown:
        if not context.launch_state.returncode:
            context.launch_state.returncode = 1

    default_exit_handler(context)


def primary_ignore_returncode_exit_handler(context):
    """
    Trigger teardown of launch and ignore return codes.

    Same as primary exit handler but ignore return codes from teardown.
    """
    if context.launch_state.teardown:
        if not context.launch_state.returncode:
            context.launch_state.returncode = 1

    default_exit_handler(context, ignore_returncode=True)


def ignore_signal_exit_handler(context):
    """
    Succeed if the process received a shutdown signal on teardown.

    Ignores return code if launch sent a SIGINT or SIGKILL to the task.
    """
    if context.launch_state.teardown:
        # Check the return code
        if context.task_state.signals_received:
            context.task_state.returncode = 0

    default_exit_handler(context)


def exit_on_error_exit_handler(context):
    """
    If the return code indicates an error trigger a tear down.

    If the return code is zero continue the other tasks.
    On Windows the task return code is ignored if launch sent a SIGINT or
    SIGKILL to the task.
    """
    if context.launch_state.teardown and context.task_state.signals_received \
            and os.name == 'nt':
        context.task_state.returncode = 0
        return

    try:
        rc = int(context.task_state.returncode)
    except (TypeError, ValueError):
        rc = 1 if bool(context.task_state.returncode) else 0
    if rc:
        # trigger tear down if not already tearing down
        if not context.launch_state.teardown:
            context.launch_state.teardown = True
        # set return code if not already set
        if not context.launch_state.returncode:
            context.launch_state.returncode = rc
