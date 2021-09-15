#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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


from enum import Enum
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import FollowWaypoints, NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class NavigationResult(Enum):
    UKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator(Node):

    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self,
                                                     NavigateThroughPoses,
                                                     'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose,
                                                        'compute_path_to_pose')
        self.compute_path_through_poses_client = ActionClient(self, ComputePathThroughPoses,
                                                              'compute_path_through_poses')
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        self.change_maps_srv = self.create_client(LoadMap, '/map_server/load_map')
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.get_costmap_global_srv = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.get_costmap_local_srv = self.create_client(GetCostmap, '/local_costmap/get_costmap')

    def setInitialPose(self, initial_pose):
        """Set the initial pose to the localization system."""
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goThroughPoses(self, poses):
        """Send a `NavThroughPoses` action request."""
        self.debug("Waiting for 'NavigateThroughPoses' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.info(f'Navigating with {len(goal_msg.poses)} goals....')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
                                                                         self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f'Goal with {len(poses)} poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def goToPose(self, pose):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followWaypoints(self, poses):
        """Send a `FollowWaypoints` action request."""
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info(f'Following {len(goal_msg.poses)} goals....')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                                                                        self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f'Following {len(poses)} waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        """Cancel pending navigation request of any type."""
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        """Check if the navigation request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Goal with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Goal succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return NavigationResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return NavigationResult.CANCELED
        else:
            return NavigationResult.UNKNOWN

    def waitUntilNav2Active(self):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def getPath(self, start, goal):
        """Send a `ComputePathToPose` action request."""
        self.debug("Waiting for 'ComputePathToPose' action server")
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal
        goal_msg.start = start

        self.info('Getting path...')
        send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f'Getting path failed with status code: {self.status}')
            return None

        return self.result_future.result().result.path

    def getPathThroughPoses(self, start, goals):
        """Send a `ComputePathThroughPoses` action request."""
        self.debug("Waiting for 'ComputePathThroughPoses' action server")
        while not self.compute_path_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathThroughPoses' action server not available, waiting...")

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.goals = goals
        goal_msg.start = start

        self.info('Getting path...')
        send_goal_future = self.compute_path_through_poses_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f'Getting path failed with status code: {self.status}')
            return None

        return self.result_future.result().result.path

    def changeMap(self, map_filepath):
        """Change the current static map in the map server."""
        while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):
            self.info('change map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath
        future = self.change_maps_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        status = future.result().result
        if status != LoadMap.Response().RESULT_SUCCESS:
            self.error('Change map request failed!')
        else:
            self.info('Change map request was successful!')
        return

    def clearAllCostmaps(self):
        """Clear all costmaps."""
        self.clearLocalCostmap()
        self.clearGlobalCostmap()
        return

    def clearLocalCostmap(self):
        """Clear local costmap."""
        while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear local costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def clearGlobalCostmap(self):
        """Clear global costmap."""
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear global costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def getGlobalCostmap(self):
        """Get the global costmap."""
        while not self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get global costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def getLocalCostmap(self):
        """Get the local costmap."""
        while not self.get_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get local costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def lifecycleStartup(self):
        """Startup nav2 lifecycle system."""
        self.info('Starting up lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Starting up {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                while True:
                    rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info('Nav2 is ready for use!')
        return

    def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system."""
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Shutting down {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
