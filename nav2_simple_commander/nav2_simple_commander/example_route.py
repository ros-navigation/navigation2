#! /usr/bin/env python3
# Copyright 2025 Open Navigation LLC
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

from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, RunningTask, TaskResult
from nav2_simple_commander.utils import euler_to_quaternion
import rclpy
from std_msgs.msg import Header

"""
Basic navigation demo to using the route server.
"""


def toPoseStamped(pt: Point, header: Header) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = pt.x
    pose.pose.position.y = pt.y
    pose.header = header
    return pose


def main() -> None:
    rclpy.init()

    node = rclpy.create_node('route_example')

    node.declare_parameter('start_pose.x', 0.0)
    node.declare_parameter('start_pose.y', 0.0)
    node.declare_parameter('start_pose.yaw', 0.0)

    node.declare_parameter('goal_pose.x', 0.0)
    node.declare_parameter('goal_pose.y', 0.0)
    node.declare_parameter('goal_pose.yaw', 0.0)

    start_x = node.get_parameter('start_pose.x').value
    start_y = node.get_parameter('start_pose.y').value
    start_yaw = node.get_parameter('start_pose.yaw').value

    goal_x = node.get_parameter('goal_pose.x').value
    goal_y = node.get_parameter('goal_pose.y').value
    goal_yaw = node.get_parameter('goal_pose.yaw').value

    node.destroy_node()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = start_x
    initial_pose.pose.position.y = start_y
    initial_pose.pose.orientation = euler_to_quaternion(0.0, 0.0, start_yaw)
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.orientation = euler_to_quaternion(0.0, 0.0, goal_yaw)

    # Sanity check a valid route exists using PoseStamped.
    # May also use NodeIDs on the graph if they are known by passing them instead as `int`
    # [path, route] = navigator.getRoute(initial_pose, goal_pose)

    # May also use NodeIDs on the graph if they are known by passing them instead as `int`
    route_tracking_task = navigator.getAndTrackRoute(initial_pose, goal_pose)

    # Note for the route server, we have a special route argument in the API b/c it may be
    # providing feedback messages simultaneously to others (e.g. controller or WPF as below)
    task_canceled = False
    last_feedback = None
    follow_path_task = RunningTask.NONE
    while not navigator.isTaskComplete(task=route_tracking_task):
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback, which contains the route / path if tracking
        feedback = navigator.getFeedback(task=route_tracking_task)
        while feedback is not None:
            if not last_feedback or \
                (feedback.last_node_id != last_feedback.last_node_id or
                    feedback.next_node_id != last_feedback.next_node_id):
                print('Passed node ' + str(feedback.last_node_id) +
                      ' to next node ' + str(feedback.next_node_id) +
                      ' along edge ' + str(feedback.current_edge_id) + '.')

            last_feedback = feedback

            if feedback.rerouted:  # or follow_path_task == RunningTask.None
                # Follow the path from the route server using the controller server
                print('Passing new route to controller!')
                follow_path_task = navigator.followPath(feedback.path)

                # May instead use the waypoint follower
                # (or nav through poses) and use the route's sparse nodes!
                # print("Passing route to waypoint follower!")
                # nodes =
                # [toPoseStamped(x.position, feedback.route.header) for x in feedback.route.nodes]
                # navigator.followWaypoints(nodes)
                # Or navigator.navigateThroughPoses(nodes)
                # Consider sending only the first few and iterating

            feedback = navigator.getFeedback(task=route_tracking_task)

        # Check if followPath or WPF task is done (or failed),
        # will cancel all current tasks, including route
        if navigator.isTaskComplete(task=follow_path_task):
            print('Controller or waypoint follower server completed its task!')
            navigator.cancelTask()
            task_canceled = True

    # Route server will return completed status before the controller / WPF server
    # so wait for the actual robot task processing server to complete
    while not navigator.isTaskComplete(task=follow_path_task) and not task_canceled:
        pass

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
