// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPENNAV_FOLLOWING__FOLLOWING_SERVER_HPP_
#define OPENNAV_FOLLOWING__FOLLOWING_SERVER_HPP_

#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include <functional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/simple_action_server.hpp"
#include "nav2_util/twist_publisher.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "opennav_docking/controller.hpp"
#include "opennav_docking/pose_filter.hpp"
#include "opennav_following_msgs/action/follow_object.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace opennav_following
{
/**
 * @class opennav_following::FollowingServer
 * @brief An action server which implements a dynamic following behavior
 */
class FollowingServer : public nav2::LifecycleNode
{
public:
  using FollowObject = opennav_following_msgs::action::FollowObject;
  using FollowingActionServer = nav2::SimpleActionServer<FollowObject>;

  /**
   * @brief A constructor for opennav_following::FollowingServer
   * @param options Additional options to control creation of the node.
   */
  explicit FollowingServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for opennav_following::FollowingServer
   */
  ~FollowingServer() = default;

  /**
   * @brief Publish feedback from a following action.
   * @param state Current state - should be one of those defined in message.
   */
  void publishFollowingFeedback(uint16_t state);

  /**
   * @brief Use control law and perception to approach the object.
   * @param object_pose Initial object pose, will be refined by perception.
   * @param target_frame The frame to be tracked instead of the pose.
   * @returns True if successfully approached, False if cancelled. For
   *          any internal error, will throw.
   */
  virtual bool approachObject(
    geometry_msgs::msg::PoseStamped & object_pose,
    const std::string & target_frame = std::string(""));

  /**
   * @brief Rotate the robot to find the object again.
   * @param object_pose The last known object pose.
   * @param target_frame The frame to be tracked instead of the pose.
   * @returns True if successful.
   */
  virtual bool rotateToObject(
    geometry_msgs::msg::PoseStamped & object_pose,
    const std::string & target_frame = std::string(""));

  /**
   * @brief Gets a preempted goal if immediately requested
   * @param Goal goal to check or replace if required with preemption
   * @param action_server Action server to check for preemptions on
   * @return SUCCESS or FAILURE
   */
  template<typename ActionT>
  void getPreemptedGoalIfRequested(
    typename std::shared_ptr<const typename ActionT::Goal> goal,
    const typename nav2::SimpleActionServer<ActionT>::SharedPtr & action_server);

  /**
   * @brief Checks and logs warning if action canceled
   * @param action_server Action server to check for cancellation on
   * @param name Name of action to put in warning message
   * @return True if action has been cancelled
   */
  template<typename ActionT>
  bool checkAndWarnIfCancelled(
    typename nav2::SimpleActionServer<ActionT>::SharedPtr & action_server,
    const std::string & name);

  /**
   * @brief Checks and logs warning if action preempted
   * @param action_server Action server to check for preemption on
   * @param name Name of action to put in warning message
   * @return True if action has been preempted
   */
  template<typename ActionT>
  bool checkAndWarnIfPreempted(
    typename nav2::SimpleActionServer<ActionT>::SharedPtr & action_server,
    const std::string & name);

  /**
   * @brief Configure member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Publish zero velocity at terminal condition
   */
  void publishZeroVelocity();

protected:
  /**
   * @brief Main action callback method to complete following request
   */
  void followObject();

  /**
   * @brief Method to obtain the refined dynamic pose.
   * @param pose The initial estimate of the dynamic pose
   *        which will be updated with the refined pose.
   * @return true if successful, false otherwise
   */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Get the pose of a specific frame in the fixed frame.
   * @param pose The output pose.
   * @param frame_id The frame to get the pose for.
   * @return true if successful, false otherwise
   */
  virtual bool getFramePose(geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id);

  /**
   * @brief Get the tracking pose based on the current tracking mode.
   * @param pose The output pose.
   * @param frame_id The frame to get the pose for.
   * @return true if successful, false otherwise.
   */
  virtual bool getTrackingPose(
    geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id);

  /**
   * @brief Get the pose at a distance in front of the input pose
   *
   * @param pose Input pose
   * @param distance Distance to move (in meters)
   * @return Pose distance meters in front of the input pose
   */
  geometry_msgs::msg::PoseStamped getPoseAtDistance(
    const geometry_msgs::msg::PoseStamped & pose, double distance);

  /**
   * @brief Check if the goal has been reached.
   *
   * @param goal_pose The goal pose to check
   * @return true If the goal has been reached
   */
  bool isGoalReached(const geometry_msgs::msg::PoseStamped & goal_pose);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Mutex for dynamic parameters
  std::mutex dynamic_params_lock_;

  // Frequency to run control loops
  double controller_frequency_;
  // Timeout to detect the object while rotating to it
  double rotate_to_object_timeout_;
  // Timeout after which a static object is considered as goal reached
  double static_object_timeout_;
  // Time when object became static
  rclcpp::Time static_object_start_time_;
  // Flag to track if we've initialized the static timer
  bool static_timer_initialized_;
  // Tolerance for transforming coordinates
  double transform_tolerance_;
  // Tolerances for arriving at the safe_distance pose
  double linear_tolerance_, angular_tolerance_;
  // Maximum number of times the robot will retry to approach the object
  int max_retries_, num_retries_;
  // This is the root frame of the robot - typically "base_link"
  std::string base_frame_;
  // This is our fixed frame for controlling - typically "odom"
  std::string fixed_frame_;
  // Desired distance to keep from the object
  double desired_distance_;
  // Skip perception orientation
  bool skip_orientation_;
  // Should the robot search for the object by rotating or go to last known heading
  bool search_by_rotating_;
  // Search angle relative to current robot orientation when rotating to find objects
  double search_angle_;

  // Timestamp of the last time a iteration was started
  rclcpp::Time iteration_start_time_;

  // This is a class member so it can be accessed in publish feedback
  rclcpp::Time action_start_time_;

  // Subscribe to the dynamic pose
  nav2::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dynamic_pose_sub_;

  // Publish the filtered dynamic pose
  nav2::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    filtered_dynamic_pose_pub_;

  // Latest message
  geometry_msgs::msg::PoseStamped detected_dynamic_pose_;

  // Filtering of detected poses
  std::unique_ptr<opennav_docking::PoseFilter> filter_;
  double detection_timeout_;

  std::unique_ptr<nav2_util::TwistPublisher> vel_publisher_;
  std::unique_ptr<nav2_util::OdomSmoother> odom_sub_;
  typename FollowingActionServer::SharedPtr following_action_server_;

  std::unique_ptr<opennav_docking::Controller> controller_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
};

}  // namespace opennav_following

#endif  // OPENNAV_FOLLOWING__FOLLOWING_SERVER_HPP_
