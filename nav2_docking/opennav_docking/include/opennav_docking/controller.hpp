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

#ifndef OPENNAV_DOCKING__CONTROLLER_HPP_
#define OPENNAV_DOCKING__CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace opennav_docking
{
/**
 * @class opennav_docking::Controller
 * @brief Default control law for approaching a dock target
 */
class Controller
{
public:
  /**
   * @brief Create a controller instance. Configure ROS 2 parameters.
   *
   * @param node Lifecycle node
   * @param tf tf2_ros TF buffer
   * @param fixed_frame Fixed frame
   * @param base_frame Robot base frame
   */
  Controller(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::shared_ptr<tf2_ros::Buffer> tf,
    std::string fixed_frame, std::string base_frame);

  /**
   * @brief A destructor for opennav_docking::Controller
   */
  ~Controller();

  /**
   * @brief Compute a velocity command using control law.
   * @param pose Target pose, in robot centric coordinates.
   * @param cmd Command velocity.
   * @param is_docking If true, robot is docking. If false, robot is undocking.
   * @param backward If true, robot will drive backwards to goal.
   * @returns True if command is valid, false otherwise.
   */
  bool computeVelocityCommand(
    const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd, bool is_docking,
    bool backward = false);

protected:
  /**
   * @brief Check if a trajectory is collision free.
   *
   * @param target_pose Target pose, in robot centric coordinates.
   * @param is_docking If true, robot is docking. If false, robot is undocking.
   * @param backward If true, robot will drive backwards to goal.
   * @return True if trajectory is collision free.
   */
  bool isTrajectoryCollisionFree(
    const geometry_msgs::msg::Pose & target_pose, bool is_docking, bool backward = false);

  /**
   * @brief Callback executed when a parameter change is detected.
   *
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Configure the collision checker.
   *
   * @param node Lifecycle node
   * @param costmap_topic Costmap topic
   * @param footprint_topic Footprint topic
   * @param transform_tolerance Transform tolerance
   */
  void configureCollisionChecker(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    std::string costmap_topic, std::string footprint_topic, double transform_tolerance);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  rclcpp::Logger logger_{rclcpp::get_logger("Controller")};
  rclcpp::Clock::SharedPtr clock_;

  // Smooth control law
  std::unique_ptr<nav2_graceful_controller::SmoothControlLaw> control_law_;
  double k_phi_, k_delta_, beta_, lambda_;
  double slowdown_radius_, v_linear_min_, v_linear_max_, v_angular_max_;

  // The trajectory of the robot while dock / undock for visualization / debug purposes
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

  // Used for collision checking
  bool use_collision_detection_;
  double projection_time_;
  double simulation_time_step_;
  double dock_collision_threshold_;
  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::string fixed_frame_, base_frame_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__CONTROLLER_HPP_
