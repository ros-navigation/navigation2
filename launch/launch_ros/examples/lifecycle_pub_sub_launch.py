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

"""Launch a lifecycle talker and a lifecycle listener."""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events

from launch_ros import get_default_launch_description
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def main(argv=sys.argv[1:]):
    """Main."""
    ld = launch.LaunchDescription()

    # Prepare the talker node.
    talker_node = launch_ros.actions.LifecycleNode(
        node_name='talker',
        package='lifecycle', node_executable='lifecycle_talker', output='screen')

    # When the talker reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_talker_reaches_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'talker' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.process.matches_action(talker_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # When the talker node reaches the 'active' state, log a message and start the listener node.
    register_event_handler_for_talker_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node, goal_state='active',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'talker' reached the 'active' state, launching 'listener'."),
                launch_ros.actions.LifecycleNode(
                    node_name='listener',
                    package='lifecycle', node_executable='lifecycle_listener', output='screen'),
            ],
        )
    )

    # Make the talker node take the 'configure' transition.
    emit_event_to_request_that_talker_does_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.process.matches_action(talker_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action(register_event_handler_for_talker_reaches_inactive_state)
    ld.add_action(register_event_handler_for_talker_reaches_active_state)
    ld.add_action(talker_node)
    ld.add_action(emit_event_to_request_that_talker_does_configure_transition)

    print('Starting introspection of launch description...')
    print('')

    print(launch.LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    # ls = launch.LaunchService(argv=argv, debug=True)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(get_default_launch_description(prefix_output_with_name=False))
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
