// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_ros_common/publisher.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost
{
  float cost{0};
  bool using_footprint{false};
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction
{
public:
  /**
    * @brief Constructor for mppi::critics::CriticFunction
    */
  CriticFunction() = default;

  /**
    * @brief Destructor for mppi::critics::CriticFunction
    */
  virtual ~CriticFunction() = default;

  /**
    * @brief Configure critic on bringup
    * @param parent WeakPtr to node
    * @param parent_name name of the controller
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    nav2::LifecycleNode::WeakPtr parent,
    const std::string & parent_name,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * param_handler)
  {
    parent_ = parent;
    auto node = parent_.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    name_ = name;
    parent_name_ = parent_name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    parameters_handler_ = param_handler;

    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(enabled_, "enabled", true);

    initialize();
  }

  /**
    * @brief Main function to score trajectory
    * @param data Critic data to use in scoring
    */
  virtual void score(CriticData & data) = 0;

  /**
    * @brief Initialize critic
    */
  virtual void initialize() = 0;

  /**
    * @brief Get name of critic
    */
  std::string getName()
  {
    return name_;
  }

protected:
  /**
    * @brief Initialize a debug pose publisher for this critic
    * @param topic Topic name suffix (e.g. "furthest_reached_path_point")
    */
  void initDebugPosePublisher(const std::string & topic)
  {
    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(debug_visualizations_, "debug_visualizations", false);
    if (debug_visualizations_) {
      auto node = parent_.lock();
      if (node) {
        // Extract short critic name from full namespaced name
        // e.g. "ctrl.PathAngleCritic" -> "PathAngleCritic"
        std::string short_name = name_;
        auto pos = short_name.rfind('.');
        if (pos != std::string::npos) {
          short_name = short_name.substr(pos + 1);
        }
        debug_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
          short_name + "/" + topic, 1);
        debug_pose_pub_->on_activate();
      }
    }
  }

  /**
    * @brief Publish a debug pose if visualization is enabled and there are subscribers
    * @param x X position
    * @param y Y position
    * @param yaw Yaw orientation
    */
  void publishDebugPose(float x, float y, float yaw)
  {
    if (debug_visualizations_ && debug_pose_pub_ &&
      debug_pose_pub_->get_subscription_count() > 0)
    {
      auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
      msg->header.frame_id = costmap_ros_->getGlobalFrameID();
      msg->header.stamp = clock_->now();
      msg->pose.position.x = x;
      msg->pose.position.y = y;
      msg->pose.position.z = 0.0;
      tf2::Quaternion quat;
      quat.setRPY(0.0, 0.0, yaw);
      msg->pose.orientation = tf2::toMsg(quat);
      debug_pose_pub_->publish(std::move(msg));
    }
  }

  bool enabled_;
  std::string name_, parent_name_;
  nav2::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  ParametersHandler * parameters_handler_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  rclcpp::Clock::SharedPtr clock_;

  bool debug_visualizations_{false};
  nav2::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_pose_pub_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
